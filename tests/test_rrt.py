from src.continuous.RRT import RRT
import numpy as np
def test_rrt():
    payload = {}
    payload['obstacles'] = [{"shape":"rectangle", "definition":[20,10,40,20]},{"shape":"circle", "definition":[10,10,3]}, {"shape":"circle", "definition":[50,50,20]}]
    payload['start'] = [0,0]
    payload['goal'] = [90,25]
    payload['goalRadius'] = 10
    payload['d_max'] = 28
    payload['width'] = 400
    payload['height'] = 400
    rrt = RRT(payload)
    assert np.linalg.norm(rrt.Target_Node.p - payload['goal']) <= payload['goalRadius']