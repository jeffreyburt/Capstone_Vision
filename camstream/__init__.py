"""Camera streaming from the Pis to the laptop: JPEG frames + metadata over ZeroMQ (see docs/NETWORK.md).

    Pi:      python -m camstream.sender robot1          (or birdseye on the master Pi)
    Laptop:  from camstream.receiver import FrameReceiver
             rx = FrameReceiver(load_config(), ["robot1", "robot2"])
             frame = rx.get_latest("robot1")            # Frame(image, seq, t_capture, meta, ...)
"""
