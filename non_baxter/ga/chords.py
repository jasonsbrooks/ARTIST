#defaults to 1, 4, 5 chords of classic 12 bar blues
def create_chord_progression():
    chord_progression = []

    for _ in range(4):
        chord_progression.append((1, 32))
    for _ in range(2):
        chord_progression.append((4, 32))
    for _ in range(2):
        chord_progression.append((1, 32))
    for _ in range(1):
        chord_progression.append((5, 32))
    for _ in range(1):
        chord_progression.append((4, 32))
    for _ in range(2):
        chord_progression.append((1, 32))

    return chord_progression


# given chord (1-7), return list of valid scale notes between 1 and 21
def get_chord_notes(chord):
    valid_notes = []
    root = chord
    third = ((chord + 1) % 7) + 1
    fifth = ((chord + 3) % 7) + 1

    multiplier = 0
    for _ in range(3):
        valid_notes.append(root + multiplier*7)
        valid_notes.append(third + multiplier*7)
        valid_notes.append(fifth + multiplier*7)

        multiplier += 1

    return valid_notes
