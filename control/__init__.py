"""Laptop-side robot control for the MVP: vision output -> speaker targets -> velocity commands.

    planner.py     listening-triangle targets around the user
    controller.py  per-robot position/heading controller (world error -> body velocity)
    link.py        UDP: vision output in, drive commands out
    main.py        the control loop (python -m control.main)
    drive_test.py  drive one robot open-loop and measure it with the camera (calibration / sign check)

The robot side is robot/drive_server.py.
"""
