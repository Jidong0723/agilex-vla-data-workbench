"""Local TCP-VLA post-processing and LeRobot export service."""
from __future__ import annotations
import hashlib, json, math, re, shutil, subprocess, tempfile, time
from bisect import bisect_left
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
import imageio_ffmpeg, numpy as np, pyarrow as pa, pyarrow.parquet as pq
from PIL import Image

APP_ROOT=Path(__file__).resolve().parent
SOURCE_ROOT=Path(r"E:\nero-agilex\dataset\episodes").resolve()
DERIVED_ROOT=Path(r"E:\nero-agilex\dataset\tcp_vla_training_views").resolve()
LEROBOT_ROOT=Path(r"E:\nero-agilex\dataset\lerobot_tcp_vla_exports").resolve()
KINEMATICS_PYTHON=Path(r"E:\nero-agilex\.conda\nero-kinematics\python.exe")
FK_WORKER=APP_ROOT/"tcp_fk_worker.py"
NAME=re.compile(r"^episode_\d+$"); HZ=20.0; PERIOD=round(1e9/HZ)
VIEW_SCHEMA="nero.tcp-vla.observation-view.v8"
COMPATIBLE_VIEW_SCHEMAS={VIEW_SCHEMA,"nero.tcp-vla.observation-view.v7"}

def read_json(path:Path)->dict: return json.loads(path.read_text(encoding="utf-8-sig"))
def read_jsonl(path:Path)->list[dict]: return [json.loads(x) for x in path.read_text(encoding="utf-8").splitlines() if x.strip()]
def write_json(path:Path,value:Any)->None:
 path.parent.mkdir(parents=True,exist_ok=True); path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
def write_jsonl(path:Path,rows:list[dict])->None:
 path.parent.mkdir(parents=True,exist_ok=True); path.write_text("".join(json.dumps(x,ensure_ascii=False,separators=(",",":"))+"\n" for x in rows),encoding="utf-8")
def finite(v:Any,n:int,name:str)->list[float]:
 if not isinstance(v,list) or len(v)!=n: raise ValueError(f"{name} must contain {n} values")
 r=[float(x) for x in v]
 if not all(math.isfinite(x) for x in r): raise ValueError(f"{name} contains non-finite values")
 return r
def source_path(name:str)->Path:
 if not NAME.fullmatch(name): raise ValueError("invalid source episode name")
 p=(SOURCE_ROOT/name).resolve()
 if p.parent!=SOURCE_ROOT or not p.is_dir(): raise ValueError("source episode was not found")
 return p
def view_path(name:str)->Path: return DERIVED_ROOT/name/"20hz_tcp_actions_v1"
def load_source(name:str)->tuple[Path,dict,list[dict],list[dict]]:
 p=source_path(name); m=read_json(p/"episode.json"); f=m.get("files") or {}
 return p,m,read_jsonl(p/str(f.get("raw_camera","raw/camera_frames.jsonl"))),read_jsonl(p/str(f.get("raw_robot_state","raw/robot_states.jsonl")))

def qnorm(q:Any)->list[float]:
 x,y,z,w=finite(q,4,"quaternion"); n=math.sqrt(x*x+y*y+z*z+w*w)
 if n<=1e-12: raise ValueError("zero quaternion")
 return [x/n,y/n,z/n,w/n]
def qmul(a:list[float],b:list[float])->list[float]:
 x,y,z,w=a; X,Y,Z,W=b
 return [w*X+x*W+y*Z-z*Y,w*Y-x*Z+y*W+z*X,w*Z+x*Y-y*X+z*W,w*W-x*X-y*Y-z*Z]
def qconj(q:list[float])->list[float]:return [-q[0],-q[1],-q[2],q[3]]
def base_tcp_delta(current:dict,following:dict)->tuple[list[float],dict]:
 p0=finite(current["position_m"],3,"current TCP position");p1=finite(following["position_m"],3,"next TCP position")
 q0=qnorm(current["orientation_xyzw"]);q1=qnorm(following["orientation_xyzw"])
 dq=qnorm(qmul(q1,qconj(q0)))
 if dq[3]<0:dq=[-x for x in dq]
 magnitude=math.sqrt(sum(x*x for x in dq[:3]))
 angle=2*math.atan2(magnitude,dq[3])
 rotvec=[x*angle/magnitude for x in dq[:3]] if magnitude>1e-12 else [0.,0.,0.]
 delta=[b-a for a,b in zip(p0,p1)]+rotvec
 theta=math.sqrt(sum(x*x for x in rotvec))
 qstep=[x*math.sin(theta/2)/theta for x in rotvec]+[math.cos(theta/2)] if theta>1e-12 else [0.,0.,0.,1.]
 reconstructed_q=qnorm(qmul(qstep,q0));residual=qnorm(qmul(q1,qconj(reconstructed_q)))
 orientation_error=2*math.atan2(math.sqrt(sum(x*x for x in residual[:3])),abs(residual[3]))
 position_error=math.sqrt(sum((a+d-b)**2 for a,d,b in zip(p0,delta[:3],p1)))
 return delta,{"position_error_m":position_error,"orientation_error_rad":orientation_error}
def qslerp(a:Any,b:Any,t:float)->list[float]:
 qa=qnorm(a);qb=qnorm(b);dot=sum(x*y for x,y in zip(qa,qb))
 if dot<0:qb=[-x for x in qb];dot=-dot
 dot=max(-1.,min(1.,dot))
 if dot>0.9995:return qnorm([(1-t)*x+t*y for x,y in zip(qa,qb)])
 theta=math.acos(dot);scale=math.sin(theta)
 return qnorm([math.sin((1-t)*theta)/scale*x+math.sin(t*theta)/scale*y for x,y in zip(qa,qb)])
def rot6d(q:list[float])->list[float]:
 x,y,z,w=qnorm(q)
 r=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
 return [r[0][0],r[1][0],r[2][0],r[0][1],r[1][1],r[2][1]]
def nearest(rows:list[dict],ts:list[int],t:int)->tuple[dict,int]:
 i=bisect_left(ts,t); choices=[j for j in (i-1,i) if 0<=j<len(ts)];j=min(choices,key=lambda k:abs(ts[k]-t));return rows[j],abs(ts[j]-t)
def interp(rows:list[dict],ts:list[int],t:int)->dict|None:
 i=bisect_left(ts,t)
 if not 0<i<len(rows):return None
 l,r=rows[i-1],rows[i]; a=(t-ts[i-1])/(ts[i]-ts[i-1])
 def vec(k,n):return [(1-a)*x+a*y for x,y in zip(finite(l.get(k),n,k),finite(r.get(k),n,k))]
 def scalar(k):
  x=float(l.get(k,l["gripper_opening_ratio"]));y=float(r.get(k,r["gripper_opening_ratio"]))
  if not math.isfinite(x+y):raise ValueError(f"{k} invalid")
  return (1-a)*x+a*y
 # A commanded gripper target is held, not linearly interpolated into a command that was never sent.
 target_source=r if t==ts[i] else l
 target_gripper=float(target_source.get("target_gripper_opening_ratio",target_source["gripper_opening_ratio"]))
 if not math.isfinite(target_gripper) or not 0<=target_gripper<=1:raise ValueError("gripper target outside [0,1]")
 return {"q":vec("joint_position_rad",7),"qd":vec("joint_velocity_rad_s",7),"g":scalar("gripper_opening_ratio"),"tg":target_gripper,"tcp":{"position_m":vec_tcp(l["fk_tcp_pose"]["position_m"],r["fk_tcp_pose"]["position_m"],a),"orientation_xyzw":qslerp(l["fk_tcp_pose"]["orientation_xyzw"],r["fk_tcp_pose"]["orientation_xyzw"],a)},"left":l,"right":r,"lt":ts[i-1],"rt":ts[i]}
def vec_tcp(left:list[float],right:list[float],alpha:float)->list[float]:return [(1-alpha)*x+alpha*y for x,y in zip(left,right)]
def fk(source:Path,meta:dict,joints:list[list[float]])->list[dict]:
 interface=meta.get("processing_interface") or {}; urdf=source/str(interface.get("urdf","raw/interface/nero_description.urdf"))
 osc=read_json(source/str(interface.get("osc_config","raw/interface/osc.json")))
 offset=finite((osc.get("tcp") or {}).get("offset_from_link7_m"),3,"TCP offset")
 p=subprocess.run([str(KINEMATICS_PYTHON),str(FK_WORKER),"--urdf",str(urdf),"--tcp-offset-json",json.dumps(offset)],input=json.dumps(joints),text=True,capture_output=True,timeout=90)
 if p.returncode:raise RuntimeError(p.stderr.strip() or "FK worker failed")
 result=json.loads(p.stdout)
 if not isinstance(result,list) or len(result)!=len(joints):raise RuntimeError("FK result count mismatch")
 for pose in result:pose["position_m"]=finite(pose.get("position_m"),3,"FK position");pose["orientation_xyzw"]=qnorm(pose.get("orientation_xyzw"));pose["orientation_rot6d"]=rot6d(pose["orientation_xyzw"])
 return result

def tcp_pair(current:dict,following:dict|None,boundary_reason:str="episode_end")->dict:
 """Preserve pose pairs and pre-label geometry for a 7D action."""
 valid=following is not None and following["target_monotonic_ns"]-current["target_monotonic_ns"]==PERIOD
 next_row=following if valid else None
 a=current["observation"];b=next_row["observation"] if next_row else None
 def pose(observation:dict)->dict:
  tcp=observation["tcp_pose"]
  return {"position_m":tcp["position_m"],"orientation_xyzw":tcp["orientation_xyzw"]}
 delta=audit=None;retry_count=0;reason="contiguous_20hz" if valid else "alignment_gap" if following is not None else boundary_reason
 if b:
  for attempt in range(2):
   try:
    candidate,error=base_tcp_delta(a["tcp_pose"],b["tcp_pose"])
    passes=all(math.isfinite(x) for x in candidate) and all(math.isfinite(x) for x in error.values())
   except (ValueError,TypeError,KeyError,OverflowError):candidate,error,passes=None,None,False
   audit=error
   if passes:
    delta=candidate;break
   retry_count+=1
  if delta is None:valid=False;reason="tcp_geometry_audit_failed"
 return {"valid":valid,"reason":reason,"current_tcp_pose":pose(a),"next_tcp_pose":pose(b) if b else None,"tcp_delta_base_6d":delta,"reconstruction_error":audit,"geometry_retry_count":retry_count,"current_target_monotonic_ns":current["target_monotonic_ns"],"next_target_monotonic_ns":next_row["target_monotonic_ns"] if next_row else None,"next_sensor_frame_index":next_row["sensor_frame_index"] if next_row else None,"current_gripper_opening_ratio":a["gripper_opening_ratio"],"next_gripper_opening_ratio":b["gripper_opening_ratio"] if b else None,"current_target_gripper_opening_ratio":a["target_gripper_opening_ratio"],"next_target_gripper_opening_ratio":b["target_gripper_opening_ratio"] if b else None}

def tcp_action(row:dict,pair:dict)->dict|None:
 if not pair["valid"]:return None
 gripper=float(row["observation"]["target_gripper_opening_ratio"])
 if not math.isfinite(gripper) or not 0<=gripper<=1:raise ValueError("gripper action outside [0,1]")
 action=[*finite(pair["tcp_delta_base_6d"],6,"TCP delta"),gripper]
 return {"tcp_delta_base":action,"delta_time_s":PERIOD/1e9,"next_sensor_frame_index":pair["next_sensor_frame_index"],"gripper_source":"recorded_target_or_observed_fallback"}

def cached_view(path:Path)->dict|None:
 try:
  view=read_json(path/"view.json")
  if view.get("schema_version") not in COMPATIBLE_VIEW_SCHEMAS:return None
  observations=read_jsonl(path/"observations.jsonl")
  if view["schema_version"]!=VIEW_SCHEMA:
   view={k:v for k,v in view.items() if k not in {"nominal_motion_limit_warnings","nominal_motion_limits","geometry_audit_tolerances","interpolated_frames","camera_held_frames","alignment_limits_s"}}
   view["threshold_filtering"]="disabled; legacy view reused without threshold judgments"
   for row in observations:
    row["alignment"].pop("camera_held",None)
    row["alignment"].pop("robot_interpolated",None)
    row["tcp_pair"].pop("nominal_motion_limit_warning",None)
  return {"view":view,"observations":observations}
 except Exception:return None
def read_view(name:str)->dict:
 p=view_path(name)
 if p.is_dir():
  cached=cached_view(p)
  if cached is not None:return cached
 return build_view(name)
def build_view(name:str)->dict:
 out=view_path(name)
 if out.is_dir():
  cached=cached_view(out)
  if cached is not None:return cached
 source,meta,camera,robot=load_source(name); b=meta["collection_monotonic_ns"];lo,hi=int(b["start"]),int(b["end"])
 streams={s:sorted([x for x in camera if x.get("source")==s and lo<=int(x["capture_monotonic_ns"])<=hi],key=lambda x:int(x["capture_monotonic_ns"])) for s in ("external","wrist")}
 robots=sorted([x for x in robot if lo<=int(x["feedback_monotonic_ns"])<=hi],key=lambda x:int(x["feedback_monotonic_ns"]))
 if not streams["external"] or not streams["wrist"] or len(robots)<2:raise ValueError("incomplete camera or robot stream")
 # Stage 1: compute FK for every original robot feedback row, before any 20 Hz resampling.
 raw_fk=fk(source,meta,[finite(x.get("joint_position_rad"),7,"joint_position_rad") for x in robots])
 for row,pose in zip(robots,raw_fk):row["fk_tcp_pose"]=pose
 cts={s:[int(x["capture_monotonic_ns"]) for x in rows] for s,rows in streams.items()};rts=[int(x["feedback_monotonic_ns"]) for x in robots]
 start=max(cts["external"][0],cts["wrist"][0],rts[0]);end=min(cts["external"][-1],cts["wrist"][-1],rts[-1]);rows=[];rejected=0
 # Keep every bracketed 20 Hz grid point; retain measured time offsets without threshold judgments.
 for t in range(start,end+1,PERIOD):
  cams={s:nearest(streams[s],cts[s],t) for s in streams};state=interp(robots,rts,t)
  if state is None:rejected+=1;continue
  bracket=max(t-state["lt"],state["rt"]-t)
  raw=state["left"] if t-state["lt"]<=state["rt"]-t else state["right"]
  pose=state["tcp"]
  pose["orientation_rot6d"]=rot6d(pose["orientation_xyzw"])
  rows.append({"sensor_frame_index":len(rows),"target_monotonic_ns":t,"prompt":str(meta.get("prompt") or ""),"images":{s:cams[s][0]["image"] for s in cams},"observation":{"joint_position_rad":state["q"],"joint_velocity_rad_s":state["qd"],"gripper_opening_ratio":state["g"],"target_gripper_opening_ratio":state["tg"],"tcp_pose":pose,"recorded_tcp_pose":raw.get("measured_tcp_pose"),"target_tcp_pose":raw.get("target_tcp_pose")},"alignment":{"raw_camera_frame_index":{s:cams[s][0]["source_frame_index"] for s in cams},"camera_error_s":{s:cams[s][1]/1e9 for s in cams},"robot_bracket_span_s":bracket/1e9,"robot_left_sample_index":state["left"]["sample_index"],"robot_right_sample_index":state["right"]["sample_index"],"robot_left_feedback_monotonic_ns":state["lt"],"robot_right_feedback_monotonic_ns":state["rt"]}})
 if not rows:raise ValueError("no valid 20 Hz observations")
 for i,row in enumerate(rows):
  row["tcp_pair"]=tcp_pair(row,rows[i+1] if i+1<len(rows) else None)
  row["action"]=tcp_action(row,row["tcp_pair"])
 view={"schema_version":VIEW_SCHEMA,"stage":"observation_with_tcp_actions","source_episode":name,"rate_hz":HZ,"sensor_frames":len(rows),"control_steps":sum(row["action"] is not None for row in rows),"adjacent_tcp_pairs":sum(row["tcp_pair"]["valid"] for row in rows),"geometry_audit_failures":sum(row["tcp_pair"]["reason"]=="tcp_geometry_audit_failed" for row in rows),"geometry_retry_attempts":sum(row["tcp_pair"]["geometry_retry_count"] for row in rows),"tcp_delta_convention":"base-frame position difference plus log(R_next * R_current^T) rotvec; not full SE(3) twist logarithm","action_order":["delta_x_base_m","delta_y_base_m","delta_z_base_m","rotvec_x_base_rad","rotvec_y_base_rad","rotvec_z_base_rad","absolute_gripper_target_ratio"],"gripper_action_source":"raw target_gripper_opening_ratio held at observation timestamp; recorder falls back to measured opening if command target missing","rejected_grid_points":rejected,"gap_filling":"20 Hz grid retains all bracketed points: robot state linear/SLERP-interpolated between raw FK samples, camera nearest-frame selected; time offsets are recorded but no time-distance or motion-amplitude threshold is applied","fk_order":"FK computed on every original robot feedback row before timestamp alignment; position linear interpolation and quaternion SLERP afterward","state_order":["q1..q7","gripper","tcp_xyz_base","tcp_rot6d_base"],"actions_generated":True,"h16_generated":False,"threshold_filtering":"disabled"}
 out.parent.mkdir(parents=True,exist_ok=True)
 with tempfile.TemporaryDirectory(prefix=name+"-",dir=out.parent) as tmp:
  p=Path(tmp)/out.name;p.mkdir();write_jsonl(p/"observations.jsonl",rows);write_json(p/"view.json",view);write_json(p/"conversion_manifest.json",{"raw_collection_immutable":True,"source_episode":name,"episode_json_sha256":hashlib.sha256((source/"episode.json").read_bytes()).hexdigest(),"view":view})
  if out.is_dir():shutil.rmtree(out)
  p.rename(out)
 return {"view":view,"observations":rows}

def split_process_at_gaps(view:dict,p:dict)->list:
 """Split only on actual time gaps or invalid TCP pairs, never on quality thresholds."""
 s,e=int(p.get("start_frame",-1)),int(p.get("end_frame",-1));obs=view["observations"]
 if s<0 or s>=len(obs):raise ValueError("process range outside derived view")
 e=min(e,len(obs)-1)
 rows=obs[s:e+1]
 segments=[];start=0
 for i in range(len(rows)-1):
  if rows[i+1]["target_monotonic_ns"]-rows[i]["target_monotonic_ns"]!=PERIOD or not rows[i]["tcp_pair"]["valid"]:
   segments.append(rows[start:i+1]);start=i+1
 segments.append(rows[start:])
 return [segment for segment in segments if segment]
def write_video(path:Path,images:list[Path])->None:
 with Image.open(images[0]) as f:size=f.convert("RGB").size
 w=imageio_ffmpeg.write_frames(str(path),size,fps=HZ,codec="libx264",pix_fmt_in="rgb24",output_params=["-pix_fmt","yuv420p","-crf","23"]);w.send(None)
 try:
  for p in images:
   with Image.open(p) as f:a=np.asarray(f.convert("RGB"),dtype=np.uint8)
   if (a.shape[1],a.shape[0])!=size:raise ValueError("camera image dimensions differ")
   w.send(a.tobytes())
 finally:w.close()
def export_root(payload:dict)->Path:
 raw=payload.get("export_directory")
 if raw is None or str(raw).strip()=="":return LEROBOT_ROOT
 path=Path(str(raw).strip().strip('"'))
 if not path.is_absolute() or not path.is_dir():raise ValueError("export directory must be an existing absolute local directory")
 path=path.resolve()
 if path==SOURCE_ROOT or SOURCE_ROOT in path.parents or path==APP_ROOT or APP_ROOT in path.parents:raise ValueError("export directory must be outside the raw episode and workbench directories")
 return path
def export_lerobot(payload:dict)->dict:
 name=str(payload.get("source_episode") or "");processes=payload.get("processes")
 if not isinstance(processes,list) or not processes:raise ValueError("at least one labeled process is required")
 outcome=str(payload.get("episode_outcome") or "unreviewed")
 if outcome not in {"unreviewed","success","failed"}:raise ValueError("invalid manual review outcome")
 source=source_path(name);source_meta=read_json(source/"episode.json");view=read_view(name);target=export_root(payload)/(name+"_tcp_vla_20hz")
 if target.exists():raise FileExistsError(f"{target.name} already exists")
 labels=payload.get("labels")
 if labels is not None:
  if not isinstance(labels,dict) or labels.get("source_view")!="20hz_tcp_actions_v1" or labels.get("processes")!=processes or labels.get("total_sensor_frames")!=view["view"]["sensor_frames"]:raise ValueError("labels do not match the 20 Hz episode view and selected processes")
 prepared=[]
 for p in processes:
  if not isinstance(p,dict) or not str(p.get("title") or "").strip():raise ValueError("each process needs a task instruction")
  for rows in split_process_at_gaps(view,p):
   if rows:prepared.append((p,rows))
 if not prepared:raise ValueError("process has no aligned observations")
 tasks=list(dict.fromkeys(str(x[0]["title"]).strip() for x in prepared));taskid={x:i for i,x in enumerate(tasks)};data=target/"data"/"chunk-000";data.mkdir(parents=True);source_index=[];pair_index=[];episodes=[];global_index=0
 for ep,(p,rows) in enumerate(prepared):
  task=str(p["title"]).strip();states=[[*r["observation"]["joint_position_rad"],r["observation"]["gripper_opening_ratio"],*r["observation"]["tcp_pose"]["position_m"],*r["observation"]["tcp_pose"]["orientation_rot6d"]] for r in rows]
  actions=[finite(r["action"]["tcp_delta_base"],7,"7D action") for r in rows[:-1]]
  actions.append([0.]*6+[float(rows[-1]["observation"]["target_gripper_opening_ratio"])])
  ex=f"videos/chunk-000/observation.images.base_0_rgb/episode_{ep:06d}.mp4";wr=f"videos/chunk-000/observation.images.left_wrist_0_rgb/episode_{ep:06d}.mp4";(target/ex).parent.mkdir(parents=True,exist_ok=True);(target/wr).parent.mkdir(parents=True,exist_ok=True)
  write_video(target/ex,[(source/r["images"]["external"]).resolve() for r in rows]);write_video(target/wr,[(source/r["images"]["wrist"]).resolve() for r in rows]);ts=[i/HZ for i in range(len(rows))];vt=pa.struct([("path",pa.string()),("timestamp",pa.float64())])
  pq.write_table(pa.table({"observation.state":pa.array(states,type=pa.list_(pa.float32(),17)),"action":pa.array(actions,type=pa.list_(pa.float32(),7)),"observation.images.base_0_rgb":pa.array([{"path":ex,"timestamp":t} for t in ts],type=vt),"observation.images.left_wrist_0_rgb":pa.array([{"path":wr,"timestamp":t} for t in ts],type=vt),"timestamp":pa.array(ts,type=pa.float64()),"frame_index":pa.array(range(len(rows)),type=pa.int64()),"episode_index":pa.array([ep]*len(rows),type=pa.int64()),"index":pa.array(range(global_index,global_index+len(rows)),type=pa.int64()),"task_index":pa.array([taskid[task]]*len(rows),type=pa.int64())}),data/f"episode_{ep:06d}.parquet",compression="zstd")
  source_index.extend({"episode_index":ep,"frame_index":i,"source_episode":name,"source_sensor_frame_index":r["sensor_frame_index"],"source_target_monotonic_ns":r["target_monotonic_ns"],"source_raw_camera_frame_index":r["alignment"]["raw_camera_frame_index"],"source_robot_left_sample_index":r["alignment"]["robot_left_sample_index"],"source_robot_right_sample_index":r["alignment"]["robot_right_sample_index"],"source_robot_left_feedback_monotonic_ns":r["alignment"]["robot_left_feedback_monotonic_ns"],"source_robot_right_feedback_monotonic_ns":r["alignment"]["robot_right_feedback_monotonic_ns"],"phase_labels":[str(tag.get("label")) for tag in p.get("tags",[]) if int(tag.get("start_frame",-1))<=r["sensor_frame_index"]<=int(tag.get("end_frame",-1))]} for i,r in enumerate(rows));episodes.append({"episode_index":ep,"tasks":[task],"length":len(rows)});global_index+=len(rows)
  pair_index.extend({"episode_index":ep,"frame_index":i,"source_sensor_frame_index":row["sensor_frame_index"],**tcp_pair(row,rows[i+1] if i+1<len(rows) else None,"segment_end")} for i,row in enumerate(rows))
 meta=target/"meta";meta.mkdir();features={"observation.state":{"dtype":"float32","shape":[17]},"action":{"dtype":"float32","shape":[7]},"observation.images.base_0_rgb":{"dtype":"video","shape":[480,640,3]},"observation.images.left_wrist_0_rgb":{"dtype":"video","shape":[480,640,3]}}
 write_json(meta/"info.json",{"codebase_version":"v2.1","robot_type":"agilex_nero","fps":HZ,"total_frames":global_index,"total_episodes":len(episodes),"total_tasks":len(tasks),"splits":{"unassigned":f"0:{len(episodes)}"},"data_path":"data/chunk-{chunk_index:03d}/episode_{episode_index:06d}.parquet","video_path":"videos/chunk-{chunk_index:03d}/{video_key}/episode_{episode_index:06d}.mp4","features":features})
 write_jsonl(meta/"tasks.jsonl",[{"task_index":i,"task":x} for i,x in enumerate(tasks)])
 write_jsonl(meta/"episodes.jsonl",episodes)
 write_jsonl(meta/"nerovla_source_index.jsonl",source_index)
 write_jsonl(meta/"nerovla_tcp_pairs.jsonl",pair_index)
 write_json(meta/"nerovla_conversion_manifest.json",{
  "dataset_stage":"observation_with_tcp_actions_intermediate",
  "tcp_geometry_generated":True,"tcp_delta_convention":view["view"]["tcp_delta_convention"],
  "actions_generated":True,"action_order":view["view"]["action_order"],
  "gripper_action_source":view["view"]["gripper_action_source"],
  "segment_terminal_action":"zero TCP delta and held absolute gripper target",
  "h16_windows_generated":False,"adjacent_tcp_pairs":sum(pair["valid"] for pair in pair_index),
  "threshold_filtering":"disabled; all measured motions and timestamp-aligned frames retained",
  "manual_review":{"outcome":outcome,"note":str(payload.get("episode_note") or "")},
  "raw_episode_prompt":str(source_meta.get("prompt") or ""),
  "raw_episode_task":str(source_meta.get("task") or ""),
  "second_postprocess_required":["review gripper target provenance","generate valid H16 anchors after segment boundaries are finalized","apply parent-group split and train-only normalization"],
  "source_episode":name,"derived_view":str(view_path(name)),"state_order":view["view"]["state_order"],
  "normalization":"deferred: calculate only after parent-group split is fixed"})
 if labels is not None:write_json(target/"labels.json",labels)
 return {"name":target.name,"path":str(target),"episodes":len(episodes),"frames":global_index,"stage":"observation_with_tcp_actions_intermediate"}

MEMORY_ROOT=DERIVED_ROOT.parent/"annotation_memory"
def slug(text:str)->str:return hashlib.sha1(text.encode()).hexdigest()[:12]
def view_anchors(name:str)->dict:
 p=view_path(name)/"observations.jsonl"
 if not p.is_file():return {"close":[],"open":[]}
 close=[];release=[];state=True
 for i,line in enumerate(p.read_text(encoding="utf-8").splitlines()):
  if not line.strip():continue
  row=json.loads(line);v=float(row["observation"]["target_gripper_opening_ratio"])
  if state and v<0.4:close.append(i);state=False
  elif not state and v>0.6:release.append(i);state=True
 return {"close":close,"open":release}
def collect_template(signature:str)->dict|None:
 rows=[]
 for f in (MEMORY_ROOT/"labels").glob("*.json"):
  archive=read_json(f)
  if archive.get("prompt")!=signature:continue
  name=archive.get("source_episode");view=view_path(name)
  if not (view/"observations.jsonl").is_file():continue
  anchors=view_anchors(name);labels=archive.get("labels") or {}
  processes=[p for p in labels.get("processes",[]) if str(p.get("title") or "").strip()]
  processes.sort(key=lambda p:int(p.get("start_frame",-1)))
  if not processes or len(anchors["close"])<len(processes):continue
  rows.append((name,anchors,processes))
 if not rows:return None
 slots=[]
 for i in range(min(len(r[2]) for r in rows)):
  starts=[];ends=[];titles={};titles_en={};phase={}
  for name,anchors,processes in rows:
   proc=processes[i]
   starts.append((int(proc["start_frame"])-anchors["close"][i])*(1000.0/HZ))
   ends.append((int(proc["end_frame"])-anchors["open"][i])*(1000.0/HZ))
   title=str(proc["title"]).strip();titles[title]=titles.get(title,0)+1
   title_en=str(proc.get("title_en") or "").strip()
   if title_en:titles_en[title_en]=titles_en.get(title_en,0)+1
   for tag in sorted(proc.get("tags",[]),key=lambda t:int(t.get("start_frame",-1))):
    lab=str(tag.get("label") or "");dur=(int(tag.get("end_frame",-1))-int(tag.get("start_frame",-1))+1)*(1000.0/HZ)
    phase.setdefault(lab,[]).append(dur)
  def stats(values):
   values=sorted(values);n=len(values)
   return {"n":n,"median_ms":round(values[n//2],1),"p25_ms":round(values[max(0,n//4)],1),"p75_ms":round(values[min(n-1,(3*n)//4)],1)}
  best_title=max(titles,key=titles.get)
  slots.append({"slot":i,"title":best_title,"title_en":max(titles_en,key=titles_en.get) if titles_en else best_title,"start_offset_from_close_ms":stats(starts),"end_offset_from_open_ms":stats(ends),"phase_durations":{lab:stats(v) for lab,v in phase.items()}})
 return {"signature":signature,"episode_count":len(rows),"process_slots":slots,"sources":[r[0] for r in rows]}
def save_labels_archive(payload:dict)->dict:
 name=str(payload.get("source_episode") or "")
 if not NAME.fullmatch(name):raise ValueError("invalid source episode name")
 labels=payload.get("labels")
 if not isinstance(labels,dict):raise ValueError("labels must be an object")
 try:prompt=str(read_json(SOURCE_ROOT/name/"episode.json").get("prompt") or "")
 except Exception:prompt=""
 archive={"source_episode":name,"prompt":prompt,"saved_at_unix_s":time.time(),"labels":labels}
 write_json(MEMORY_ROOT/"labels"/f"{name}.json",archive)
 template=collect_template(prompt)
 if template:write_json(MEMORY_ROOT/"templates"/f"{slug(prompt)}.json",template)
 return {"saved":True,"template_episodes":template["episode_count"] if template else 0}
def get_template(payload:dict)->dict:
 name=str(payload.get("source_episode") or "")
 prompt=str(read_json(SOURCE_ROOT/name/"episode.json").get("prompt") or "")
 f=MEMORY_ROOT/"templates"/f"{slug(prompt)}.json"
 if not f.is_file():return {"found":False}
 return {"found":True,"template":read_json(f)}
def clear_memory()->dict:
 removed=0
 for sub in ("labels","templates"):
  d=MEMORY_ROOT/sub
  if d.is_dir():
   for f in d.glob("*.json"):f.unlink();removed+=1
 return {"cleared":True,"removed":removed}
# 任意文件夹导入：浏览器把拖入的 episode 数据分批上传到 SOURCE_ROOT，之后走统一管道
def import_manifest(payload:dict)->list[tuple[str,int]]:
 files=payload.get("files")
 if not isinstance(files,list) or not files or len(files)>100_000:raise ValueError("invalid import file list")
 result=[];seen=set()
 for item in files:
  if not isinstance(item,dict):raise ValueError("invalid import file entry")
  rel=item.get("path");size=item.get("size")
  if not isinstance(rel,str) or not rel or rel.startswith("/") or "\\" in rel or ":" in rel or any(part in ("",".","..") for part in rel.split("/")):raise ValueError("invalid import file path")
  if rel in seen or not isinstance(size,int) or size<0:raise ValueError("invalid import file size or duplicate path")
  seen.add(rel);result.append((rel,size))
 if "episode.json" not in seen:raise ValueError("import is missing episode.json")
 return result
def episode_exists(payload:dict)->dict:
 name=str(payload.get("name") or "")
 if not NAME.fullmatch(name):raise ValueError("invalid episode name")
 files=import_manifest(payload);target=SOURCE_ROOT/name;metadata=payload.get("metadata")
 if not isinstance(metadata,dict):raise ValueError("missing episode metadata")
 existing=target/"episode.json"
 try:existing_meta=read_json(existing) if existing.is_file() else None
 except (OSError,ValueError):existing_meta=None
 conflict=existing_meta is not None and existing_meta!=metadata
 missing=[rel for rel,size in files if not (target/rel).is_file() or (target/rel).stat().st_size!=size or (rel=="episode.json" and existing_meta is None)]
 return {"exists":target.is_dir(),"complete":target.is_dir() and not conflict and not missing,"conflict":conflict,"missing":missing}
def import_begin(payload:dict)->dict:
 name=str(payload.get("name") or "")
 if not NAME.fullmatch(name):raise ValueError("invalid episode name")
 target=SOURCE_ROOT/name;meta=payload.get("metadata");existing=target/"episode.json"
 if not isinstance(meta,dict):raise ValueError("missing episode metadata")
 try:existing_meta=read_json(existing) if existing.is_file() else None
 except (OSError,ValueError):existing_meta=None
 if existing_meta is not None and existing_meta!=meta and not payload.get("overwrite"):raise ValueError("same-name episode has different metadata; confirm overwrite")
 target.mkdir(parents=True,exist_ok=True)
 return {"begin":True}
def parse_multipart(body:bytes,boundary:bytes):
 out=[]
 for part in body.split(b"--"+boundary):
  if part in (b"",b"--",b"--\r\n"):continue
  head,sep,content=part.partition(b"\r\n\r\n")
  if not sep:continue
  if content.endswith(b"\r\n"):content=content[:-2]
  disp=""
  for line in head.split(b"\r\n"):
   k,_,v=line.partition(b":")
   if k.strip().lower()==b"content-disposition":disp=v.strip().decode()
  m=re.search(r'name="path:([^"]*)"',disp)
  if m:out.append((m.group(1),content))
 return out
def import_files(handler)->dict:
 name=handler.headers.get("X-Episode-Name","")
 if not NAME.fullmatch(name):raise ValueError("invalid episode name")
 ctype=handler.headers.get("Content-Type","")
 if not ctype.startswith("multipart/form-data") or "boundary=" not in ctype:raise ValueError("expected multipart/form-data")
 boundary=ctype.split("boundary=")[-1].strip().encode()
 n=int(handler.headers.get("Content-Length","0"))
 if not 0<n<=500_000_000:raise ValueError("invalid upload size")
 body=handler.rfile.read(n)
 target=SOURCE_ROOT/name;written=skipped=0;overwrite=handler.headers.get("X-Overwrite")=="1"
 for rel,content in parse_multipart(body,boundary):
  if not rel or rel.startswith("/") or "\\" in rel or ":" in rel or any(part in ("",".","..") for part in rel.split("/")):raise ValueError("bad file path")
  p=(target/rel)
  if p.parent!=target and target not in p.parents:raise ValueError("bad file path")
  p.parent.mkdir(parents=True,exist_ok=True)
  if not overwrite and p.is_file() and p.stat().st_size==len(content) and (rel!="episode.json" or p.read_bytes()==content):skipped+=1;continue
  p.write_bytes(content);written+=1
 return {"written":written,"skipped":skipped}
def import_end(payload:dict)->dict:
 name=str(payload.get("name") or "")
 if not NAME.fullmatch(name):raise ValueError("invalid episode name")
 files=import_manifest(payload);target=SOURCE_ROOT/name;metadata=payload.get("metadata")
 if not isinstance(metadata,dict) or not (target/"episode.json").is_file() or read_json(target/"episode.json")!=metadata:raise ValueError("import incomplete: episode.json mismatch")
 missing=[rel for rel,size in files if not (target/rel).is_file() or (target/rel).stat().st_size!=size]
 if missing:raise ValueError(f"import incomplete: {len(missing)} files missing or wrong size")
 return {"done":True,"files":len(files)}

class Handler(SimpleHTTPRequestHandler):
 def __init__(self,*a,**kw):super().__init__(*a,directory=str(APP_ROOT),**kw)
 def send_json(self,status:int,v:Any):
  b=json.dumps(v,ensure_ascii=False).encode();self.send_response(status);self.send_header("Content-Type","application/json; charset=utf-8");self.send_header("Content-Length",str(len(b)));self.end_headers();self.wfile.write(b)
 def do_POST(self):
  if self.path not in {"/api/build-tcp-view","/api/export-lerobot","/api/save-labels","/api/get-template","/api/clear-memory","/api/episode-exists","/api/import-begin","/api/import-files","/api/import-end"}:self.send_error(HTTPStatus.NOT_FOUND);return
  try:
   if self.path=="/api/import-files":self.send_json(200,import_files(self));return
   n=int(self.headers.get("Content-Length","0"))
   if not 0<n<=2_000_000:raise ValueError("invalid request body")
   p=json.loads(self.rfile.read(n).decode())
   if self.path=="/api/build-tcp-view":out=build_view(str(p.get("source_episode") or ""))
   elif self.path=="/api/export-lerobot":out={"created":export_lerobot(p)}
   elif self.path=="/api/save-labels":out=save_labels_archive(p)
   elif self.path=="/api/get-template":out=get_template(p)
   elif self.path=="/api/clear-memory":out=clear_memory()
   elif self.path=="/api/episode-exists":out=episode_exists(p)
   elif self.path=="/api/import-begin":out=import_begin(p)
   else:out=import_end(p)
   self.send_json(200,out)
  except FileExistsError as e:self.send_json(409,{"error":str(e)})
  except (ValueError,TypeError,KeyError,RuntimeError,json.JSONDecodeError,subprocess.TimeoutExpired) as e:self.send_json(400,{"error":str(e)})
  except Exception as e:self.send_json(500,{"error":f"local conversion failed: {type(e).__name__}"})
if __name__=="__main__":ThreadingHTTPServer(("127.0.0.1",8790),Handler).serve_forever()
