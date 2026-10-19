#!/usr/bin/python3
"""Install only the reviewed SDK Python sources, never its bundled Windows venv."""
import hashlib
from pathlib import Path
import site
import urllib.request
import zipfile
import io

URL = 'https://files.waveshare.com/wiki/Bus_Servo_Adapter_A/STServo_Python.zip'
SHA256 = 'f39b7a07324b68571071ce8b36a8e1ded4da5215b77b883ea601dc8e7a21172b'


def main():
    data = urllib.request.urlopen(URL, timeout=60).read()
    if hashlib.sha256(data).hexdigest() != SHA256:
        raise RuntimeError('Vendor archive changed; review API and checksum before installing')
    destination = Path(site.getusersitepackages()) / 'scservo_sdk'
    if destination.exists():
        raise RuntimeError(f'{destination} already exists; inspect it before replacing')
    prefix = 'STServo_Python/stservo-env/scservo_sdk/'
    with zipfile.ZipFile(io.BytesIO(data)) as archive:
        sources = {Path(n).name: archive.read(n) for n in archive.namelist()
                   if n.startswith(prefix) and '/' not in n[len(prefix):] and n.endswith('.py')}
    if 'sms_sts.py' not in sources:
        raise RuntimeError('Missing sms_sts API')
    destination.mkdir(parents=True)
    for name, contents in sources.items():
        (destination / name).write_bytes(contents)
    print(destination)


if __name__ == '__main__':
    main()
