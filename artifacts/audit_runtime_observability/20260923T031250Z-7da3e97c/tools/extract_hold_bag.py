"""Audit-only typed PX4 protocol extraction; bag stamp is observer wall time."""
import csv
from pathlib import Path
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    bag,out=map(Path,sys.argv[1:3]);run=sys.argv[3]
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),
                rosbag2_py.ConverterOptions(input_serialization_format='cdr',output_serialization_format='cdr'))
    types={x.name:get_message(x.type) for x in reader.get_all_topics_and_types()}
    rows=[]
    while reader.has_next():
        topic,data,observer_ns=reader.read_next()
        if topic=='/clock':continue
        msg=deserialize_message(data,types[topic])
        fields=dict(run=run,topic=topic,bag_observer_wall_ns=observer_ns,px4_boot_us=getattr(msg,'timestamp',''),
                    command=getattr(msg,'command',''),nav_state=getattr(msg,'nav_state',''),
                    result=getattr(msg,'result',''),param1=getattr(msg,'param1',''),
                    source_system=getattr(msg,'source_system',''),source_component=getattr(msg,'source_component',''),
                    target_system=getattr(msg,'target_system',''),target_component=getattr(msg,'target_component',''),
                    from_external=getattr(msg,'from_external',''),executor_in_charge=getattr(msg,'executor_in_charge',''),
                    failsafe=getattr(msg,'failsafe',''),mission_id=getattr(msg,'mission_id',''),
                    waypoint_index=getattr(msg,'waypoint_index',''),request_id=getattr(msg,'request_id',''))
        rows.append(fields)
    with out.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]),lineterminator='\n');w.writeheader();w.writerows(rows)
    from collections import Counter
    print('rows',len(rows),'topics',dict(Counter(x['topic'] for x in rows)))


if __name__=='__main__':main()
