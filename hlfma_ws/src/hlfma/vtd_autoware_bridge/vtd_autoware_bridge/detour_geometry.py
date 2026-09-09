"""Conservative planar footprint checks; no ROS graph or third-party geometry dependency.

Candidate motion is sampled in arc length. Convex swept hulls include a rotation
padding; dynamic prediction envelopes supplement (never replace) Autoware RSS.
"""
import math


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


def hull(points):
    points = sorted(set(points))
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    sides = []
    for seq in (points, list(reversed(points))):
        side = []
        for p in seq:
            while len(side) >= 2 and cross(side[-2], side[-1], p) <= 0:
                side.pop()
            side.append(p)
        sides.extend(side[:-1])
    return sides


def overlaps(a, b):
    if len(a) < 3 or len(b) < 3:
        return True  # Missing geometry must not grant clearance.
    for poly in (a, b):
        for p, q in zip(poly, poly[1:] + poly[:1]):
            nx, ny = p[1]-q[1], q[0]-p[0]
            av = [x*nx+y*ny for x, y in a]
            bv = [x*nx+y*ny for x, y in b]
            if max(av) < min(bv) or max(bv) < min(av):
                return False
    return True


def transform(points, x, y, heading):
    c, s = math.cos(heading), math.sin(heading)
    return [(x+c*a-s*b, y+s*a+c*b) for a, b in points]


def footprint(x, y, heading, front, rear, width, margin=0.0):
    return transform([(front+margin, width/2+margin),
                      (front+margin, -width/2-margin),
                      (-rear-margin, -width/2-margin),
                      (-rear-margin, width/2+margin)], x, y, heading)


def object_polygon(obj, pose=None):
    pose = pose or obj.kinematics.initial_pose_with_covariance.pose
    shape = obj.shape
    if shape.type == shape.POLYGON:
        local = [(p.x, p.y) for p in shape.footprint.points]
    elif shape.dimensions.x > 0 and shape.dimensions.y > 0:
        # A bounding box also conservatively encloses cylinders.
        x, y = shape.dimensions.x/2, shape.dimensions.y/2
        local = [(x,y), (x,-y), (-x,-y), (-x,y)]
    else:
        return []
    if not all(math.isfinite(v) for pair in local for v in pair):
        return []
    if not all(math.isfinite(v) for v in (pose.position.x, pose.position.y, yaw(pose.orientation))):
        return []
    return hull(transform(local, pose.position.x, pose.position.y, yaw(pose.orientation)))


class ArcPath:
    def __init__(self, points):
        self.xy = []
        self.s = []
        for p in points:
            xy = (p.pose.position.x, p.pose.position.y)
            if not all(math.isfinite(v) for v in xy):
                raise ValueError('non-finite path')
            d = math.dist(xy, self.xy[-1]) if self.xy else 0.0
            if self.xy and d < 1e-6:
                continue
            self.xy.append(xy)
            self.s.append((self.s[-1] if self.s else 0.0) + d)
        if len(self.xy) < 2:
            raise ValueError('short path')
        self.length = self.s[-1]

    def project(self, x, y):
        best = (math.inf, 0.0)
        for i, (a, b) in enumerate(zip(self.xy, self.xy[1:])):
            d = self.s[i+1]-self.s[i]
            t = max(0., min(1., ((x-a[0])*(b[0]-a[0])+(y-a[1])*(b[1]-a[1]))/d**2))
            best = min(best, (math.hypot(x-a[0]-t*(b[0]-a[0]), y-a[1]-t*(b[1]-a[1])), self.s[i]+t*d))
        return best

    def at(self, s):
        s = max(0., min(self.length, s))
        for i in range(len(self.s)-1):
            if self.s[i+1] >= s:
                a, b = self.xy[i:i+2]
                t = (s-self.s[i])/(self.s[i+1]-self.s[i])
                return a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]), math.atan2(b[1]-a[1], b[0]-a[0])

    def samples(self, start, end, step):
        # Include original vertices so no corner is cut by resampling.
        values = {start, end}
        values.update(s for s in self.s if start < s < end)
        for i in range(1, int((end-start)/step)+1):
            values.add(min(end, start+i*step))
        return [(s, self.at(s)) for s in sorted(values)]


def object_id(obj):
    """Return a stable identifier for collision diagnostics and filtering."""
    return bytes(getattr(getattr(obj, 'object_id', None), 'uuid', []))


def clearance_with_collision(path, ego, objects, front, rear, width, margin, lookahead, step=0.5,
                             prediction_horizon=0.0):
    """Return arc clearance and the colliding object's ID, if any."""
    _, origin = path.project(ego.x, ego.y)
    end = min(path.length, origin+lookahead)
    polygons = []
    for obj in objects:
        oid = object_id(obj)
        polygon = object_polygon(obj)
        if not polygon:
            return 0.0, oid
        polygons.append((polygon, oid))
        for predicted in obj.kinematics.predicted_paths:
            dt = predicted.time_step.sec + predicted.time_step.nanosec*1e-9
            if prediction_horizon <= 0 or dt <= 0:
                continue
            previous = polygon
            previous_yaw = yaw(obj.kinematics.initial_pose_with_covariance.pose.orientation)
            for i, pose in enumerate(predicted.path):
                if i*dt > prediction_horizon:
                    break
                current = object_polygon(obj, pose)
                if not current:
                    return 0.0, oid
                angle = abs((yaw(pose.orientation)-previous_yaw+math.pi) % (2*math.pi)-math.pi)
                radius = max(math.dist(a, b) for a in current for b in current)
                pad = radius*angle
                swept = hull([(x+dx, y+dy) for x, y in previous+current
                              for dx, dy in ((pad,pad), (pad,-pad), (-pad,pad), (-pad,-pad))])
                previous_yaw = yaw(pose.orientation)
                polygons.append((swept, oid))
                previous = current
    # Keep IDs while deduplicating identical static predictions.
    polygons = list({(tuple(poly), oid): (poly, oid) for poly, oid in polygons}.values())
    boxes = [(min(x for x,y in poly), min(y for x,y in poly),
              max(x for x,y in poly), max(y for x,y in poly), poly, oid)
             for poly, oid in polygons]
    samples = path.samples(origin, end, step)
    for (s0, p0), (_, p1) in zip(samples, samples[1:]):
        angle = abs((p1[2]-p0[2]+math.pi) % (2*math.pi)-math.pi)
        padding = math.hypot(max(front, rear), width/2)*angle
        swept = hull(footprint(*p0, front, rear, width, margin+padding) +
                     footprint(*p1, front, rear, width, margin+padding))
        xmin, ymin = min(x for x,y in swept), min(y for x,y in swept)
        xmax, ymax = max(x for x,y in swept), max(y for x,y in swept)
        for a, b, c, d, poly, oid in boxes:
            if c >= xmin and a <= xmax and d >= ymin and b <= ymax and overlaps(swept, poly):
                return max(0.0, s0-origin), oid
    return max(0.0, end-origin), None


def clearance(path, ego, objects, front, rear, width, margin, lookahead, step=0.5,
              prediction_horizon=0.0):
    """Backward-compatible distance-only corridor clearance."""
    distance, _ = clearance_with_collision(
        path, ego, objects, front, rear, width, margin, lookahead, step, prediction_horizon)
    return distance
