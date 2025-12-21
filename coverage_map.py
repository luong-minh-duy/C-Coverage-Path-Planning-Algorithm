def get_sampling_front(discovered_old, discovered_new):
    """
    Sampling front = biên của vùng đã phát hiện
    """
    if discovered_new is None or discovered_new.is_empty:
        return discovered_new

    # lấy biên của vùng đã biết
    front = discovered_new.boundary

    # buffer mỏng để thành polygon (vì boundary là LineString)
    return front.buffer(1e-3)
