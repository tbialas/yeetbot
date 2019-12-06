import cv2
import sys
from shapely import geometry as geom

class Human:
    def __init__(self, humanid, detection_box, frame):
        self.humanid = humanid
        self.detection_box = detection_box
        self.tracker = cv2.TrackerMOSSE_create()
        print(self.detection_box)
        xmin, ymin = self.detection_box[0]
        xmax, ymax = self.detection_box[1]
        self.tracker.init(frame, (xmin, ymin, (xmax-xmin), (ymax-ymin)))
        self.box = detection_box
        self.last_seen = cv2.getTickCount()
        self.dead = False
        self.trackerbox = None


    def update_tracking(self, frame, detection_boxes):
        ok, bbox = self.tracker.update(frame)
        if (ok):
            trackbox = list()
            trackbox.append((int(bbox[0]), int(bbox[1])))
            trackbox.append( (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3])) )
            self.trackerbox = trackbox
            max_inter = 0
            maxmatch = None
            matches = list()
            for detection_box in detection_boxes:
                inter = self.calc_inter(detection_box, trackbox)

                print("OVERLAP IS {0}".format(inter))
                if (inter > 0.50):
                    matches.append(detection_box)
                if (inter > max_inter):
                    max_inter = inter
                    maxmatch = detection_box
            if (max_inter > 0.50):
                print("Tracker matches detection box!")
                self.box = maxmatch
                (xmin, ymin) = detection_box[0]
                (xmax, ymax) = detection_box[1]
                if (max_inter < 0.7):
                    self.tracker = None
                    self.tracker = cv2.TrackerMOSSE_create()
                    self.tracker.init(frame, (xmin, ymin, (xmax-xmin), (ymax-ymin)))
            else:
                print("No detection box matches tracker!")
                self.box = trackbox
            self.last_seen = cv2.getTickCount()
            return matches
        else:
            self.trackerbox = None
            print("Tracker failure!")
            max_inter = 0
            maxmatch = None
            matches = list()
            for detection_box in detection_boxes:
                inter = self.calc_inter(detection_box, self.box)
                print("OVERLAP IS {0}".format(inter))

                if (inter > 0.50):
                    matches.append(detection_box)
                if (inter > max_inter):
                    max_inter = inter
                    maxmatch = detection_box
            if (max_inter > 0.50):
                print("Tracker matches detection box!")
                self.box = maxmatch
                self.last_seen = cv2.getTickCount()
                (xmin, ymin) = detection_box[0]
                (xmax, ymax) = detection_box[1]
                self.tracker = None
                self.tracker = cv2.TrackerMOSSE_create()
                self.tracker.init(frame, (xmin, ymin, (xmax-xmin), (ymax-ymin)))

            else:
                print("No detection box matches box!")
                
            return matches



    def calc_inter(self, box1, box2):
        (box1_minx, box1_miny) = box1[0]
        (box1_maxx, box1_maxy) = box1[1]
        (box2_minx, box2_miny) = box2[0]
        (box2_maxx, box2_maxy) = box2[1]
        poly1 = geom.box(box1_minx, box1_miny, box1_maxx, box1_maxy)
        poly2 = geom.box(box2_minx, box2_miny, box2_maxx, box2_maxy)
        #inter = poly1.intersection(poly2).area / poly1.union(poly2).area
        inter = poly1.intersection(poly2).area / poly2.area
        return inter

        
