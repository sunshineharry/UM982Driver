
def check_checksum(nmea_sentence):
    # 移除起始的'$'和'*'及之后的校验和部分
    try:
        sentence, checksum = nmea_sentence[1:].split("*")
    except:
        return False
    calculated_checksum = 0
    # 对字符串中的每个字符进行异或运算
    for char in sentence:
        calculated_checksum ^= ord(char)
    # 将计算得到的校验和转换为十六进制格式，并大写
    calculated_checksum_hex = format(calculated_checksum, 'X')
    # 校验和比较
    return calculated_checksum_hex.zfill(2) == checksum.upper()




def check_crc(nmea_sentence):
    # create crc_table
    def crc_table():
        table = []
        for i in range(256):
            crc = i
            for j in range(8, 0, -1):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xEDB88320
                else:
                    crc >>= 1
            table.append(crc)
        return table

    # Function to calculate CRC using the provided algorithm and the generated table
    def calculate_crc32(data):
        table = crc_table()
        crc = 0
        for byte in data:
            crc = table[(crc ^ byte) & 0xFF] ^ (crc >> 8)
        return crc & 0xFFFFFFFF

    # main check_crc
    try:
        sentence, crc = nmea_sentence[1:].split("*")
    except:
        return False
    return crc == format(calculate_crc32(sentence.encode()), '08x')


def msg_seperate(msg:str):
    return msg[1:msg.find('*')].split(',')


def determine_utm_zone_and_hemisphere(lat, lon):
    """
    Determines the UTM zone and hemisphere (north or south) for a given latitude and longitude.

    Parameters:
    lat (float): Latitude of the point.
    lon (float): Longitude of the point.

    Returns:
    tuple: A tuple containing the UTM zone number and a boolean indicating whether it's in the northern hemisphere.
    """
    # UTM zones are determined by longitude, starting at -180 degrees.
    zone_number = int((lon + 180) / 6) + 1

    # Northern hemisphere is above the equator (latitude 0)
    north = lat >= 0

    return zone_number, north
