import numpy as np

color_dic_hsv = {
    "red1": [(0, 120, 70),(10, 255, 255)],
    "red2": [(170, 120,70),(180,255,255)],
    "orange":[(15,120,70),(25,255,255)],
    "yellow":[(20,120,70),(35,255,255)],
    "green":[(45,120,70),(70,255,255)],
    "blue":[(90,120,70),(120,255,255)],
    "purple":[(135,120,70),(145,255,255)],
}

color_dic_bgr = {
    "bright_red": np.array([46, 51, 239]),
    "dark_red": np.array([36, 39, 190]),

    "bright_orange": np.array([57, 164, 246]),
    "dark_orange": np.array([44, 140, 210]),

    "bright_yellow": np.array([39, 224, 230]),
    "dark_yellow": np.array([43, 210, 214]),

    "bright_green": np.array([57, 161, 61]),
    "dark_green": np.array([27, 103, 30]),

    "bright_blue": np.array([227, 163, 55]),
    "dark_blue": np.array([186, 135, 44]),

    "bright_purple": np.array([208, 42, 153]),
    "dark_purple": np.array([181, 41, 131]),
}
