import unittest
import xml.etree.ElementTree as ET

from engineering_inventory import table, inspect


class NativeTableGrammar(unittest.TestCase):
    def test_file_inventory_records_real_product_digest(self):
        import tempfile
        from pathlib import Path
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'fixture.xml'
            path.write_text('<root><table><tableData>0 2\n1 3</tableData></table></root>')
            result=inspect(path)
            self.assertEqual(len(result),1)
            self.assertTrue(result[0]['passed'])
            self.assertEqual(len(result[0]['file_sha256']),64)

    def test_internal_two_dimensional_without_independent_vars(self):
        result=table(ET.fromstring('<table type="internal"><tableData>0 1\n0 2 3\n1 4 5</tableData></table>'))
        self.assertEqual(result['dimension'],2)
        self.assertEqual(result['shape'],[2,2])

    def test_internal_one_dimensional_remains_one_dimensional(self):
        result=table(ET.fromstring('<table type="internal"><tableData>0 2\n1 3</tableData></table>'))
        self.assertEqual(result['dimension'],1)

    def test_missing_column_is_rejected(self):
        with self.assertRaisesRegex(ValueError,'column'):
            table(ET.fromstring('<table><tableData>0 1\n0 2 3\n1 4</tableData></table>'))

    def test_duplicate_axis_is_rejected(self):
        with self.assertRaisesRegex(ValueError,'axis'):
            table(ET.fromstring('<table><tableData>0 1\n0 2 3\n0 4 5</tableData></table>'))

    def test_nonfinite_value_is_rejected(self):
        with self.assertRaisesRegex(ValueError,'nonfinite'):
            table(ET.fromstring('<table><tableData>0 1\n1 nan</tableData></table>'))

    def test_declared_dimension_mismatch_is_rejected(self):
        with self.assertRaisesRegex(ValueError,'dimension'):
            table(ET.fromstring('<table><independentVar lookup="row">x</independentVar><tableData>0 1\n0 2 3\n1 4 5</tableData></table>'))

    def test_three_dimensional_slice_grid_is_checked(self):
        node=ET.fromstring('<table><tableData breakPoint="0">0 1\n0 2 3\n1 4 5</tableData><tableData breakPoint="1">0 1\n0 2 3\n1 4 5</tableData></table>')
        self.assertEqual(table(node)['dimension'],3)
        node.findall('tableData')[1].set('breakPoint','0')
        with self.assertRaisesRegex(ValueError,'axis'):table(node)


if __name__=='__main__':unittest.main()
