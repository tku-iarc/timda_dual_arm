#!/usr/bin/env python3

class ObjInfo(dict):
    def __init__(self): #, pre_id, name, letter, expire_state, id):

        self.side_id_name = ['front', 'back', 'left_side', 'right_side', 'bottom', 'side_id5', 'side_id6', 'side_id7', 'side_id8', 'side_id9']
        self.obj = {
            # predefined merchandise information
                "pre_id": None,     # predefined aruco ID range(i.e. 10~19)
                "name": '',         # 'plum_riceball', 'salmon_riceball', 'sandwich', 'burger', 'drink', 'lunch_box'
                "letter": '',       # ABCD, EFGH, IJK, LMN, OPQ, RST
                "expired": '',      # 'new', 'old', 'expired'

            # update after camera take picture, changable due to object pose
                "id": None,         # current detected aruco ID
                "side_id": '',      # side_id_name: 'front', 'back', 'left_side', 'right_side', 'bottom', ...

                "pos": None,        # position (x, y, z)                
                "vector": None,     # aruco marker z axis vector????
                "euler": None,      # rotation
                "sucang": None,     # sucker angle?

                "cam_H_mrk": None,
        }
     
    def __getitem__(self, key):
        return self.obj[key]

    def __setitem__(self, key, value):
        self.obj[key] = value
