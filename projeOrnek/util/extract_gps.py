from PIL import Image
from PIL.ExifTags import TAGS, GPSTAGS

def get_exif_data(image_path):
    """Returns a dictionary from the exif data of an PIL Image item."""
    try:
        image = Image.open(image_path)
        exif_data = image._getexif()
        if exif_data is not None:
            return exif_data
    except Exception as e:
        print(f"Error reading EXIF data: {e}")
    return None

def _get_if_exist(data, key):
    if key in data:
        return data[key]
    return None

def _convert_to_degress(value):
    """Converts GPS coordinates stored in EXIF format to degrees."""
    d = float(value[0])
    m = float(value[1])
    s = float(value[2])
    return d + (m / 60.0) + (s / 3600.0)

def get_lat_lon_alt(exif_data):
    """Returns the latitude, longitude and altitude, if available, from the provided exif_data (obtained through get_exif_data)."""
    lat = None
    lon = None
    alt = None

    if not exif_data:
        return None, None, None

    gps_info = {}
    for (key, val) in exif_data.items():
        if TAGS.get(key) == "GPSInfo":
            for (t, v) in val.items():
                decode = GPSTAGS.get(t, t)
                gps_info[decode] = v
            break

    gps_latitude = _get_if_exist(gps_info, 'GPSLatitude')
    gps_latitude_ref = _get_if_exist(gps_info, 'GPSLatitudeRef')
    gps_longitude = _get_if_exist(gps_info, 'GPSLongitude')
    gps_longitude_ref = _get_if_exist(gps_info, 'GPSLongitudeRef')
    gps_altitude = _get_if_exist(gps_info, 'GPSAltitude')
    
    if gps_latitude and gps_latitude_ref and gps_longitude and gps_longitude_ref:
        lat = _convert_to_degress(gps_latitude)
        if gps_latitude_ref != "N":
            lat = 0 - lat

        lon = _convert_to_degress(gps_longitude)
        if gps_longitude_ref != "E":
            lon = 0 - lon
            
    if gps_altitude:
        alt = float(gps_altitude)

    return lat, lon, alt
