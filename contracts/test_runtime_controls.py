"""Mirrored runtime schema fixtures; run with the existing contract test dependencies."""
import json
from pathlib import Path
import unittest
from jsonschema import Draft202012Validator

ROOT=Path(__file__).resolve().parent


class RuntimeSchemaTests(unittest.TestCase):
    def test_requests(self):
        schema=json.loads((ROOT/'runtime-controls.schema.json').read_text())
        Draft202012Validator.check_schema(schema)
        validator=Draft202012Validator(schema)
        fixtures=json.loads((ROOT/'runtime-controls.fixtures.json').read_text())
        for value in fixtures['valid']:
            with self.subTest(value=value):self.assertEqual(list(validator.iter_errors(value)),[])
        for value in fixtures['invalid']:
            with self.subTest(value=value):self.assertTrue(list(validator.iter_errors(value)))

    def test_response(self):
        schema=json.loads((ROOT/'runtime-controls.response.schema.json').read_text())
        Draft202012Validator.check_schema(schema)
        validator=Draft202012Validator(schema)
        fixtures=json.loads((ROOT/'runtime-controls.fixtures.json').read_text())
        for value in fixtures['responses']:
            self.assertEqual(list(validator.iter_errors(value)),[])
            value['snapshot']['manual']['confirmed']={'linear_m_s':.1}
            self.assertTrue(list(validator.iter_errors(value)))


if __name__=='__main__':unittest.main()
