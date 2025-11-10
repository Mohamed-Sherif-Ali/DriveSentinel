#!/usr/bin/env python3
import argparse, cv2, numpy as np, time, sys

def parse_args():
    p = argparse.ArgumentParser(description="Simple Lane Detector")
    p.add_argument("--cam", type=int, default=0)
    p.add_argument("--width", type=int, default=320)
    p.add_argument("--height", type=int, default=240)
    p.add_argument("--show", action="store_true", help="Display frames")
    return p.parse_args()

def roi_mask(img):
    h, w = img.shape[:2]
    mask = np.zeros_like(img)
    # Lower half
    pts = np.array([[(0, h),(0, h//2),(w, h//2),(w, h)]], dtype=np.int32)
    cv2.fillPoly(mask, pts, 255)
    return cv2.bitwise_and(img, mask)

def decide(midpoints, frame_center):
    if not midpoints: 
        return "UNKNOWN"
    m = int(np.mean(midpoints[-10:]))
    if m < frame_center - 10: return "LEFT"
    if m > frame_center + 10: return "RIGHT"
    return "CENTER"

def main():
    args = parse_args()
    cap = cv2.VideoCapture(args.cam)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    mids = []
    while True:
        ok, frame = cap.read()
        if not ok: 
            time.sleep(0.05); 
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        edges = cv2.Canny(blur, 80, 160)
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT,(3,3)))
        masked = roi_mask(edges)
        lines = cv2.HoughLinesP(masked, 1, np.pi/180, threshold=40, minLineLength=30, maxLineGap=25)
        left_x, right_x = [], []
        h, w = frame.shape[:2]
        frame_center = w//2
        if lines is not None:
            for x1,y1,x2,y2 in lines[:,0]:
                slope = (y2-y1)/(x2-x1+1e-6)
                if slope < -0.5:
                    left_x += [x1, x2]
                elif slope > 0.5:
                    right_x += [x1, x2]
        midpoint = None
        if left_x and right_x:
            midpoint = int((max(left_x)+min(right_x))//2)
            mids.append(midpoint)
            mids = mids[-30:]

        status = decide(mids, frame_center)

        if args.show:
            vis = frame.copy()
            if midpoint is not None:
                cv2.line(vis, (midpoint, 0), (midpoint, h), (0,255,0), 2)
            cv2.line(vis, (frame_center, 0), (frame_center, h), (255,0,0), 1)
            cv2.putText(vis, f"LANE:{status}", (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
            cv2.imshow("lane", vis)
            if cv2.waitKey(1) == 27: break
        else:
            print(f"LANE:{status}")
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
