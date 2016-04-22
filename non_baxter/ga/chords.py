#defaults to 1, 4, 5 chords of classic 12 bar blues
def create_chord_progression(a_train=False):
    chord_progression = []

    if not a_train:
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
    else:
        for _ in range(2):  # 16 measures
            for _ in range(2):
                chord_progression.append((1, 32))
            for _ in range(2):
                chord_progression.append((2, 32))
            for _ in range(1):
                chord_progression.append((3, 32))
            for _ in range(1):
                chord_progression.append((4, 32))
            for _ in range(2):
                chord_progression.append((1, 32))

        for _ in range(4):  # 8 measures
            chord_progression.append((5, 32))
        for _ in range(2):
            chord_progression.append((4, 32))
        for _ in range(1):
            chord_progression.append((3, 32))
        for _ in range(1):
            chord_progression.append((6, 32))

        for _ in range(2):  # 8 measures
            chord_progression.append((1, 32))
        for _ in range(2):
            chord_progression.append((2, 32))
        for _ in range(1):
            chord_progression.append((3, 32))
        for _ in range(1):
            chord_progression.append((6, 32))
        for _ in range(2):
            chord_progression.append((1, 32))

    return chord_progression



# given chord (1-7), return list of valid scale notes between 1 and 21
def get_chord_notes(chord, a_train=False):
    if a_train:
        chord_to_notes = {
            1: [1, 3, 5],  # C
            2: [1, 2],  # D7b5
            3: [2, 4],  # D_7
            4: [1, 2, 6],  # D7
            5: [1, 4, 6],  # F
            6: [2, 4, 5, 7],  # G7
            7: [1, 2, 3, 5]  # C9
        }

        valid_notes = []
        for note in chord_to_notes[chord]:
            valid_notes.append(note)
            valid_notes.append(note + 7)
            valid_notes.append(note + 14)

        return valid_notes

    else:
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
