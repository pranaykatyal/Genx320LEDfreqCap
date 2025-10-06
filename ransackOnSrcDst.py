import cv2 as cv
import numpy as np
import random as rand


src1 = [(65, 8), (75, 11), (84, 15), (91, 35), (102, 40), (106, 41), (114, 59), (120, 62), (124, 63)]
dst1 = [(163, 106), (178, 113), (191, 118), (203, 151), (215, 158), (226, 162), (238, 187), (248, 194), (258, 196)]


src2 = [(48, 18), (57, 21),(62, 22), (73, 38), (77, 40),(82, 43), (90, 57), (96, 60),(99, 60)]
dst2 = [(133, 112), (144, 117), (155, 121), (166, 146), (175, 151), (185, 154), (196, 174), (204, 180), (213, 182)]

src3 = [(52, 31), (56, 33),(60, 34), (64, 42), (68, 44),  (72, 46), (76, 53), (80, 55) ,(84,56)]
dst3 = [(132, 126), (138, 128), (144, 131), (150, 144), (156, 146), (161, 149), (168, 160), (172, 162), (178, 164)]


src4 = [(46, 27), (51, 28), (55, 29), (62, 40), (66, 43), (71, 44), (77, 54), (82, 57), (88, 59)]
dst4 =[(137, 110), (145, 113), (152, 115), (160, 133), (167, 137), (174, 140), (184, 154), (191, 159), (199, 162)]


src5 = [(46, 14),(53, 16),(61, 18),(69, 35),(77, 39),(86, 42),(97, 59),(106, 64),(115, 66)]
dst5 =[(130, 115), (142, 120), (154, 123), (167, 152), (180, 159), (193, 163), (209, 189), (222, 198), (237, 202)]


src6 = [(20, 18),(24, 20),(29, 20),(36, 37),(41, 39),(48, 42),(57, 59),(65, 64),(75, 67)]
dst6 =[(79, 110), (90, 115), (100, 118), (111, 146), (122, 152), (134, 157), (148, 184), (160, 192), (174, 197)]


src7 = [(48, 29),(53, 30),(57, 32),(61, 42),(65, 44),(70, 46),(76, 56),(81, 59),(86, 60)]
dst7 =[(128, 126), (134, 128), (141, 130), (148, 146), (154, 150), (161, 153), (169, 168), (176, 173), (184, 176)]


src8 = [(69, 34),(72, 35),(76, 36),(80, 44),(83, 46),(87, 47),(92, 54),(95, 57),(99, 58)]
dst8 =[(158, 143), (163, 145), (168, 147), (173, 159), (178, 162), (184, 164), (190, 174), (196, 178), (201, 179)]


src9 = [(58, 35),(61, 36),(65, 38),(68, 45),(71, 46),(75, 47),(79, 53),(82, 55),(85, 56)]
dst9 =[(141, 128), (146, 130),(150, 132), (156, 142), (160, 146),(165, 147), (171, 156),  (175, 159) ,(180, 160)]


srclist = [src1, src2, src3, src4, src5, src6, src7, src8, src9]
dstlist = [dst1, dst2, dst3, dst4, dst5, dst6, dst7, dst8, dst9]


def process(srclist, dstlist):
    if len(dstlist) != len(srclist):
        print("Not correct input")
        return []
    
    srclist.sort(key=lambda x: x[0])
    dstlist.sort(key=lambda x: x[0])
    
    mainlist = []
    for i in range(len(srclist)):
        mainlist.append((np.array(srclist[i]), np.array(dstlist[i])))
    
    return mainlist

def ransac_homography(mainlist, threshold=5.0, iterations=1000):
    best_H = None
    best_inliers = []
    
    for _ in range(iterations):
        # Randomly sample 4 correspondences (minimum required for homography)
        sample = rand.sample(mainlist, 4)
        src_pts = np.array([p[0] for p in sample], dtype=np.float32)
        dst_pts = np.array([p[1] for p in sample], dtype=np.float32)

        # Compute homography from 4 points
        H, _ = cv.findHomography(src_pts, dst_pts, 0)
        if H is None:
            continue
        
        # Count inliers
        inliers = []
        for s, d in mainlist:
            p_src = np.array([s[0], s[1], 1.0])
            p_proj = H @ p_src
            p_proj /= p_proj[2]

            error = np.linalg.norm(p_proj[:2] - d)
            print(error,"error")
            if error < threshold:
                inliers.append((s, d))

        # Update best model if this one has more inliers
        if len(inliers) > len(best_inliers):
            best_inliers = inliers
            best_H = H

    # Recompute final homography using all inliers
    if len(best_inliers) >= 4:
        final_src = np.array([p[0] for p in best_inliers], dtype=np.float32)
        final_dst = np.array([p[1] for p in best_inliers], dtype=np.float32)
        final_H, _ = cv.findHomography(final_src, final_dst, 0)
        return final_H
    else:
        print("Not enough inliers found.")
        return None
    

# Accumulate all correspondences in a list
big = []
for i in range(len(srclist)):
    big.extend(process(srclist[i], dstlist[i]))


print(big, "output")
H = ransac_homography(big)
print("Homography Matrix:", H)

# Use H to transform all src lists and compare to their dst
if H is not None:
    for idx in range(len(srclist)):
        src_pts = np.array(srclist[idx], dtype=np.float32)
        dst_pts = np.array(dstlist[idx], dtype=np.float32)
        src_hom = np.hstack([src_pts, np.ones((src_pts.shape[0], 1))])
        proj = (H @ src_hom.T).T
        proj = proj / proj[:, 2][:, np.newaxis]
        proj_xy = proj[:, :2]
        print(f"\nProjected src points using H for src{idx+1}:")
        for i, (p, d) in enumerate(zip(proj_xy, dst_pts)):
            print(f"src: {src_pts[i]}  -> projected: {p.round(2)}  |  dst: {d}")
