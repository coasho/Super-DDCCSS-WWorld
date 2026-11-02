"""Artifact admission/refusal checks; these are not physical-model tests."""
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from acceptance import digest, regression
from contract import METHOD_TESTS


class RegressionAdmission(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(prefix='sim-m01-admission-',dir='D:/ENV/tmp')
        self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name)
        self.junit=self.root/'junit.xml';self.receipt=self.root/'regression.json'
        self.names=list(METHOD_TESTS)

    def write(self,names=None,failed=False,skip=False,inventory=None):
        suite=ET.Element('testsuite')
        for name in self.names if names is None else names:
            case=ET.SubElement(suite,'testcase',name=name)
            if failed:ET.SubElement(case,'failure')
            if skip:ET.SubElement(case,'skipped')
        ET.ElementTree(suite).write(self.junit,encoding='utf-8')
        data=dict(passed=True,junit=str(self.junit),junit_sha256=digest(self.junit),
            executed=len(suite),elapsed_s=.01,inventory=dict(tests=[dict(name=n) for n in self.names if inventory is None] if inventory is None else inventory))
        self.receipt.write_text(json.dumps(data),encoding='utf-8')

    def test_complete_evidence_is_admitted(self):
        self.write();self.assertEqual(regression(self.receipt)['executed'],len(self.names))

    def test_truncation_is_refused(self):
        self.write(names=self.names[:-1])
        with self.assertRaises(ValueError):regression(self.receipt)

    def test_failure_cannot_be_overridden_by_summary_boolean(self):
        self.write(failed=True)
        with self.assertRaises(ValueError):regression(self.receipt)

    def test_skip_cannot_be_counted_as_passed(self):
        self.write(skip=True)
        with self.assertRaises(ValueError):regression(self.receipt)

    def test_changed_raw_receipt_is_refused(self):
        self.write();self.junit.write_bytes(self.junit.read_bytes()+b' ')
        with self.assertRaises(ValueError):regression(self.receipt)

    def test_missing_method_is_refused_even_if_inventory_agrees(self):
        self.write(names=self.names[:-1],inventory=[dict(name=n) for n in self.names[:-1]])
        with self.assertRaises(ValueError):regression(self.receipt)

    def test_duplicate_test_names_are_refused(self):
        self.write(names=self.names+[self.names[0]])
        with self.assertRaises(ValueError):regression(self.receipt)


if __name__=='__main__':unittest.main()
