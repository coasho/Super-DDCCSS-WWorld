"""A removed generated section cannot masquerade as a current model product."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from hangar import pieces


class PiecePublication(unittest.TestCase):
    def test_removed_sections_retire_only_their_owned_xml(self):
        with tempfile.TemporaryDirectory(dir='D:/ENV/tmp') as temporary:
            root=Path(temporary)
            owned={section:root/'pieces'/pieces.piece_name('fixture',section)/
                   (pieces.piece_name('fixture',section)+'.xml') for section in pieces.PARTS}
            for path in owned.values():
                path.parent.mkdir(parents=True);path.write_text('old',encoding='utf-8')
            neighbor=owned['tail'].with_name('user-notes.txt');neighbor.write_text('preserve',encoding='utf-8')
            unrelated=root/'pieces'/'unrelated.xml';unrelated.write_text('preserve',encoding='utf-8')
            with patch.object(pieces,'piece_xml',side_effect=lambda _data,section:'current\n' if section=='nose' else None):
                result=pieces.write({'aircraft':'fixture'},str(root))
            self.assertEqual(result,[str(owned['nose'])])
            self.assertEqual(owned['nose'].read_text(encoding='utf-8'),'current\n')
            for section in pieces.PARTS:
                if section!='nose':self.assertFalse(owned[section].exists())
            self.assertEqual(neighbor.read_text(encoding='utf-8'),'preserve')
            self.assertEqual(unrelated.read_text(encoding='utf-8'),'preserve')

    def test_invalid_design_identity_refuses_before_retirement(self):
        with tempfile.TemporaryDirectory(dir='D:/ENV/tmp') as temporary:
            root=Path(temporary);sentinel=root/'keep.xml';sentinel.write_text('preserve',encoding='utf-8')
            for identity in ('../foreign','..','a/b','a\\b','D:foreign',''):
                with self.subTest(identity=identity),self.assertRaises(ValueError):
                    pieces.write({'aircraft':identity},str(root))
            self.assertEqual(sentinel.read_text(encoding='utf-8'),'preserve')


if __name__=='__main__':unittest.main(verbosity=2)
