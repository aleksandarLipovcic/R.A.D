"""render_labels.py -- draw per-frame YOLO labels on the video (class names only).
python render_labels.py --video rad_fpv_cropped_part0.mp4 --labels labels/part0 --out part0_labeled.mp4"""
import argparse, re, subprocess
from pathlib import Path
import cv2
CL=["person","car","large_vehicle","motorcycle","other_vehicle"]
COL=[(0,0,255),(0,200,0),(255,128,0),(0,200,255),(255,0,255)]
p=argparse.ArgumentParser(); p.add_argument("--video",required=True); p.add_argument("--labels",required=True)
p.add_argument("--out",required=True); p.add_argument("--crf",type=int,default=20); a=p.parse_args()
files={int(re.findall(r"\d+",f.stem)[-1]):f for f in Path(a.labels).glob("*.txt")}
cap=cv2.VideoCapture(a.video); fps=cap.get(5); W,H=int(cap.get(3)),int(cap.get(4))
enc=subprocess.Popen(["ffmpeg","-v","error","-y","-f","rawvideo","-pix_fmt","bgr24","-s",f"{W}x{H}","-r",str(fps),"-i","-",
    "-c:v","libx264","-preset","veryfast","-crf",str(a.crf),"-pix_fmt","yuv420p","-movflags","+faststart",a.out],stdin=subprocess.PIPE)
i=0
while True:
    ok,f=cap.read()
    if not ok: break
    for line in (files[i].read_text().split("\n") if i in files else []):
        t=line.split()
        if len(t)<5: continue
        c=int(float(t[0])); xc,yc,w,h=map(float,t[1:5])
        x1,y1,x2,y2=int((xc-w/2)*W),int((yc-h/2)*H),int((xc+w/2)*W),int((yc+h/2)*H)
        cv2.rectangle(f,(x1,y1),(x2,y2),COL[c],2)
        (tw,th),_=cv2.getTextSize(CL[c],cv2.FONT_HERSHEY_SIMPLEX,0.5,1); ty=y1-4 if y1-th-6>0 else y2+th+4
        cv2.rectangle(f,(x1,ty-th-3),(x1+tw+4,ty+3),COL[c],-1)
        cv2.putText(f,CL[c],(x1+2,ty),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1,cv2.LINE_AA)
    enc.stdin.write(f.tobytes()); i+=1
enc.stdin.close(); enc.wait(); print("rendered",i,"frames ->",a.out)
