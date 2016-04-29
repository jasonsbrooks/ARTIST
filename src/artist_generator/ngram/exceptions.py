class InvalidKeySignature(Exception):
    """
    Exception to raise when we encounter an invalid key signature.
    """
    def __init__(self):
        Exception.__init__(self)