import rospy,rospkg,os

# calculate directory location information
_rospack = rospkg.RosPack()

ARTIST_PKG_DIR = _rospack.get_path('baxter_artist')
ARTIST_SHARE_DIR = os.path.join(ARTIST_PKG_DIR,'share/')

IMAGE_PATH = os.path.join(ARTIST_SHARE_DIR,'images/')
CONFIG_FILENAME = os.path.join(ARTIST_SHARE_DIR,'conf.json')
KEYS_FILENAME = os.path.join(ARTIST_SHARE_DIR,'keys.json')

# export BaxterController, Leaner. must stay after rospack lines above!!
from .controller import BaxterController
from .learner import Learner
from .performer import Performer