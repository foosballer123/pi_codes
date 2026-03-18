# pin dictionary for motors and encoders
# uses minimal descriptive language for readability when called in other codes
# L and R correspond with "Left" and "Right" sides of the table based on Blue Player facing 



motor = {

    1: {
        "DESCRIPTION": "Linear motion of the Blue Players nearest Home Goal.\n Uses two optical encoders ( SENSOR L/R ) for measurements. \n Pins encoded as PUL, DIR, and SENSOR.",
        "DIR": 17,
        "PUL": 27,
        "SENSOR L": 5,          # encoder 2 in encstats
        "SENSOR R": 6       # encoder 1 in encstats
    },
    2: {
        "DESCRIPTION": "Angular motion of the Blue Players nearest Home Goal.\n Uses one optical encoder for measurements. \n Pins encoded as PUL, DIR, and SENSOR.",
        "DIR": 19,
        "PUL": 26,
        "SENSOR": 4    # encoder 3 in encstats
    },
    3: {
        "DESCRIPTION": "Linear motion of the Blue Players nearest Away Goal.\n Uses two optical encoders ( SENSOR L/R ) for measurements. \n Pins encoded as PUL, DIR, and SENSOR.",
        "DIR": 20,
        "PUL": 21,
        "SENSOR L": 8,      # encoder 5 in encstats
        "SENSOR R": 7       # encoder 4 in encstats
    },
    4: {
        "DESCRIPTION": "Angular motion of the Blue Players nearest Away Goal.\n Uses one optical encoder for measurements.\n Pins encoded as PUL, DIR, and SENSOR.",
        "DIR": 23,
        "PUL": 24,
        "SENSOR": 2      # encoder 6 in encstats
    }
}
