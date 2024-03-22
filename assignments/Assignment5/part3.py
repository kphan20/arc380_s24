import compas_rrc as rrc


def gripper_on():
    rrc.SetDigital("DO00", 1)
    rrc.WaitTime(2)  # TODO see if these are the right settings to wait


def gripper_off():
    rrc.SetDigital("DO00", 0)
    rrc.WaitTime(2)


def sort_pieces():
    pass


if __name__ == "__main__":
    sort_pieces()
