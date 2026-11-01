"""Discovery must reject excluded directory names before accessing their contents."""
import unittest
from unittest.mock import patch

from hangar import __main__, storestages


class StoreDiscovery(unittest.TestCase):
    def test_excluded_directories_are_never_probed(self):
        def allowed_probe(path):
            self.assertNotIn('.codex', str(path).lower().replace('\\', '/').split('/'))
            return False

        for discover in (storestages.designs,
                         lambda: storestages.carrier_for({'parameters': {'mass_kg': {'value': 1}}, 'class': 'fixture', 'id': 'fixture'}),
                         lambda: __main__.main(['list'])):
            with self.subTest(discover=discover), \
                 patch.object(storestages.os, 'listdir', return_value=['.codex', '.CoDeX', 'fixture']), \
                 patch.object(storestages.os.path, 'isfile', side_effect=allowed_probe):
                discover()


if __name__ == '__main__':
    unittest.main()
