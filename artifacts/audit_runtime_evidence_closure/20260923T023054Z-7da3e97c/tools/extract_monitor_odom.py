"""Extract propagated odometry from runner monitor into replay CSV.

The first column is monitor arrival_wall_ns, not ROS bag recorder time or the
MissionController callback timestamp. Both wall observations share a host clock
but their event order near a callback remains uncertain.
"""
import csv
import json
from pathlib import Path
import sys


def main():
    run,out=map(Path,sys.argv[1:3])
    with (run/"perception_timeline.jsonl").open() as source,out.open("w",newline="") as target:
        writer=csv.writer(target,lineterminator="\n")
        writer.writerow(["bag_ns","source_ns","epoch","sequence","x","y","z","vx","vy","vz"])
        n=0
        for line in source:
            r=json.loads(line)["record"]
            if r.get("stream")!="propagated_odometry" or not r.get("accepted_by_monitor"):
                continue
            p=r["payload"]
            writer.writerow([r["arrival_wall_ns"],p["stamp_ns"],p["localization_epoch"],p["sequence"],*p["position"],*p["linear_velocity"]])
            n+=1
    print(n)


if __name__=="__main__":main()
