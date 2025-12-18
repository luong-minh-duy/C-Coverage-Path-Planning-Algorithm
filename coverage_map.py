def get_sampling_front(discovered_old, discovered_new):
    """
    Tính vùng sampling front: phần mới được phát hiện (discovered_new - discovered_old).
    """
    if discovered_old is None:
        return discovered_new
    else:
        return discovered_new.difference(discovered_old)