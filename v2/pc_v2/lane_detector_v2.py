#!/usr/bin/env python3
import argparse, cv2, numpy as np, time

def parse_args():
    p = argparse.ArgumentParser(description="Lane Detector v2")
    p.add_argument("--cam", type=int, default=0)
    p.add_argument("--show", action="store_true")
    return p.parse_args()

def decide(midpoints, center):
    if not midpoints:
        return "UNKNOWN"
    m = int(np.mean(midpoints[-15:]))
    if m < center - 10: return "LEFT"
    if m > center + 10: return "RIGHT"
    return "CENTER"

def main():
    args = parse_args()
    cap = cv2.VideoCapture(args.cam)
    mids = []
    while True:
        ok, frame = cap.read()
        if not ok: time.sleep(0.05)
continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        edges = cv2.Canny(blur, 80, 160)
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT,(3,3)))
        h,w = frame.shape[:2]
center = w//2
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=40, minLineLength=30, maxLineGap=25)
        left_x, right_x = [], []
        if lines is not None:
            for x1,y1,x2,y2 in lines[:,0]:
                slope = (y2-y1)/(x2-x1+1e-6)
                if slope < -0.5: left_x += [x1,x2]
                elif slope > 0.5: right_x += [x1,x2]
        midpoint = None
        if left_x and right_x:
            midpoint = int((max(left_x)+min(right_x))//2)
            mids.append(midpoint)
mids = mids[-30:]

        status = decide(mids, center)
        if args.show:
            vis = frame.copy()
            if midpoint is not None:
                cv2.line(vis, (midpoint,0), (midpoint,h), (0,255,0), 2)
            cv2.line(vis, (center,0), (center,h), (255,0,0), 1)
            cv2.putText(vis, f"LANE:{status}", (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
            cv2.imshow("lane_v2", vis)
            if cv2.waitKey(1) == 27: break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
