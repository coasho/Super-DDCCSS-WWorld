"""The Earth's clouds for the viewer: NASA's Blue Marble cloud map, one grey channel.

    python tools/data/cloud_map/make_cloud_map.py [--tif cloud_combined_8192.tif]

Downloads NASA Earth Observatory's "Blue Marble: Clouds" (cloud_combined_8192.tif,
8192 x 4096, equirectangular, 36 MB; a public-domain work of the US Government) unless
given it, and writes assets/maps/clouds/earth-clouds-8192.png: its one channel
(the three are the same grey), 0 clear .. 255 the brightest cloud, the top row at
90 N, the left column at 180 W. The viewer reads it for the cloud cover over the
whole planet (world::Clouds): where the weather systems are, and what the Earth
looks like from far off.
"""
import argparse
import os
import sys
import urllib.request

URL = "https://eoimages.gsfc.nasa.gov/images/imagerecords/57000/57747/cloud_combined_8192.tif"
ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
OUT = os.path.join(ROOT, "assets", "maps", "clouds", "earth-clouds-8192.png")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--tif", help="the downloaded cloud_combined_8192.tif (fetched when not given)")
    args = parser.parse_args()
    try:
        from PIL import Image
    except ImportError:
        sys.exit("needs Pillow: python -m pip install pillow")
    Image.MAX_IMAGE_PIXELS = None
    tif = args.tif
    if not tif:
        tif = os.path.join(os.path.dirname(OUT), "cloud_combined_8192.tif")
        if not os.path.exists(tif):
            os.makedirs(os.path.dirname(tif), exist_ok=True)
            print("downloading", URL)
            urllib.request.urlretrieve(URL, tif)
    image = Image.open(tif)
    if image.size != (8192, 4096):
        sys.exit("expected 8192 x 4096, got %d x %d" % image.size)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    image.convert("L").save(OUT, optimize=True)
    print("wrote", OUT, "(%.1f MB)" % (os.path.getsize(OUT) / 1e6))
    if not args.tif:
        os.remove(tif)


if __name__ == "__main__":
    main()
