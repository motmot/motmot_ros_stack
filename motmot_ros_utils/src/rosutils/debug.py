import logging

def enable_console_logging(debug):
    #also redirect logging to stderr
    console = logging.StreamHandler()
    level = logging.DEBUG if debug else logging.INFO
    console.setLevel(level)
    formatter = logging.Formatter('%(name)-12s: %(levelname)-8s %(message)s')
    console.setFormatter(formatter)
    logging.getLogger('').addHandler(console)
