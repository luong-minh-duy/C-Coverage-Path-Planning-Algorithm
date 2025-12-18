
def update_discovered(discovered, new_scan):
    """
    Cập nhật vùng đã phát hiện bởi union với vùng vừa quét được new_scan.
    """
    if discovered is None:
        return new_scan
    else:
        return discovered.union(new_scan)