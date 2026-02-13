# Message to publish available color names and their RGB values
# This message is published to provide clients with a list of available colors

string[] color_names     # List of available color names
uint8[] r_values         # Red values corresponding to each color name
uint8[] g_values         # Green values corresponding to each color name
uint8[] b_values         # Blue values corresponding to each color name
string[] descriptions    # Optional descriptions for each color
