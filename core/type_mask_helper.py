def build_type_mask(
    ignore_position=False,
    ignore_velocity=False,
    ignore_acceleration=False,
    ignore_yaw=False,
    ignore_yaw_rate=False
) -> int:
    mask = 0b000000000000

    # Pozisyon
    if ignore_position:
        mask |= 0b000000000111

    # Hız
    if ignore_velocity:
        mask |= 0b000000111000

    # İvme
    if ignore_acceleration:
        mask |= 0b000111000000

    # Yaw
    if ignore_yaw:
        mask |= 0b010000000000

    # Yaw rate
    if ignore_yaw_rate:
        mask |= 0b100000000000

    return mask

if __name__ == "__main__":
    print(f"Type Mask: {bin(build_type_mask(ignore_position=True, ignore_acceleration=True))}")