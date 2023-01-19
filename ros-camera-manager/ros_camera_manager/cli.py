#!/bin/python3 

import rclpy
import argparse
import client 
import tabulate

def main ():
    parser = argparse.ArgumentParser()

    sp = parser.add_subparsers(dest="command")
    list_parser = sp.add_parser("list")

    info_parser = sp.add_parser("info")
    info_parser.add_argument("id")

    args = parser.parse_args()
    
    c_client = client.Client()

    if args.command == "list":
        resp = c_client.status()
        data = []
        for camera in resp.cameras:
            data.append([camera.id, camera.driver_type, len(camera.streams), camera.available])

        print(tabulate.tabulate(data, headers=["id", "driver", "stream count", "available"]))
    
    elif args.command == "info":
        res = c_client.queryById(args.id)
        cam = None

        if len(res.cameras):
            cam = res.cameras[0]
        else:
            return

        print(cam)

if __name__ == "__main__":
    rclpy.init()
    main()
        





