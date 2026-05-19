import re
from logging import getLogger
from typing import Optional

logger = getLogger(__name__)

# Matches both patterns:
#   com.flytrex.bms-gX.vY.Z.T-rcA-B
#   com.flytrex.bms-gX.vY.Z.T
_FW_VERSION_PATTERN = re.compile(
    rb'com\.flytrex\.bms-g(\d+)\.v(\d+)\.(\d+)\.(\d+)(?:-rc(\d+)-(\d+))?'
)


class FirmwareBinaryParser:
    """
    @brief          Parse a .bin firmware file and extract version metadata
    @               from the embedded version string.
    @               Searches the binary for a string matching:
    @                   com.flytrex.bms-gX.vY.Z.T          (official build)
    @                   com.flytrex.bms-gX.vY.Z.T-rcA-B    (release candidate)
    @               Example:
    @                   com.flytrex.bms-g3.v0.8.2-rc5-2
    @                       hardware_type = "g3"
    @                       version       = "v0.8.2"
    @                       rc            = 5
    @                       sub_rc        = 2
    @                       is_official   = False
    """

    def __init__(self, bin_path: str):
        """
        @brief          Initialise the parser and immediately parse the given file.
        @param[in]      bin_path    Path to the .bin firmware file.
        """
        self._bin_path = bin_path
        self._hw_type = None     # e.g. "g3"
        self._version = None     # e.g. "v0.8.2"
        self._rc = None          # e.g. 5, or None if official
        self._sub_rc = None      # e.g. 2, or None if official
        self._match_found = False
        self._full_match = None  # the full matched string

        self._parse()

    def _parse(self):
        """
        @brief          Read the binary file and search for the version string.
        """
        try:
            with open(self._bin_path, 'rb') as f:
                data = f.read()
        except Exception as ex:
            logger.error('FirmwareBinaryParser: could not read file %s: %s', self._bin_path, ex)
            return

        match = _FW_VERSION_PATTERN.search(data)
        if match is None:
            logger.warning('FirmwareBinaryParser: version string not found in %s', self._bin_path)
            return

        self._match_found = True
        self._full_match = match.group(0).decode('ascii', errors='replace')

        g_num = match.group(1).decode('ascii')
        v_major = match.group(2).decode('ascii')
        v_minor = match.group(3).decode('ascii')
        v_patch = match.group(4).decode('ascii')

        self._hw_type = 'g' + g_num
        self._version = 'v%s.%s.%s' % (v_major, v_minor, v_patch)

        if match.group(5) is not None and match.group(6) is not None:
            self._rc = int(match.group(5))
            self._sub_rc = int(match.group(6))

        logger.info('FirmwareBinaryParser: found "%s" in %s', self._full_match, self._bin_path)

    @property
    def match_found(self) -> bool:
        """
        @brief          Check whether the version string was found in the binary.
        @return         True if found, False otherwise.
        """
        return self._match_found

    @property
    def full_match(self) -> Optional[str]:
        """
        @brief          Get the full matched version string.
        @return         The matched string (e.g. "com.flytrex.bms-g3.v0.8.2-rc5-2"),
                        or None if not found.
        """
        return self._full_match

    def hardware_type(self) -> Optional[str]:
        """
        @brief          Get the hardware type string.
        @return         Hardware type (e.g. "g3"), or None if not found.
        """
        return self._hw_type

    def version(self) -> Optional[str]:
        """
        @brief          Get the version string.
        @return         Version (e.g. "v0.8.2"), or None if not found.
        """
        return self._version

    def rc(self) -> Optional[int]:
        """
        @brief          Get the release candidate number.
        @return         RC number (e.g. 5), or None if official or not found.
        """
        return self._rc

    def sub_rc(self) -> Optional[int]:
        """
        @brief          Get the sub release candidate number.
        @return         Sub-RC number (e.g. 2), or None if official or not found.
        """
        return self._sub_rc

    def is_match(self, version_string: str) -> bool:
        """
        @brief          Check whether the given string equals the extracted
                        version pattern.
        @param[in]      version_string  String to compare against the full
                                        matched version (e.g.
                                        "com.flytrex.bms-g3.v0.8.2-rc5-2").
        @return         True if equal, False otherwise (including when no
                        match was found in the binary).
        """
        if not self._match_found or self._full_match is None:
            return False
        return version_string == self._full_match

    def is_official(self) -> bool:
        """
        @brief          Check whether the build is an official release
                        (no -rcA-B suffix).
        @return         True if official, False otherwise.
                        Returns False if the version string was not found.
        """
        if not self._match_found:
            return False
        return self._rc is None
