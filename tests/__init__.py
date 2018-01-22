import logging
import os

# force use of mavlink v2.0
os.environ['MAVLINK20'] = '1'

logging.getLogger(__name__).addHandler(logging.NullHandler())
