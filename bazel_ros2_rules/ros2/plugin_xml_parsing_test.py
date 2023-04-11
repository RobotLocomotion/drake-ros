import tempfile
import unittest

from ros2bzl.scraping.metadata import parse_plugins_description_xml


class PluginXmlParsingTest(unittest.TestCase):

    def test_one_library(self):
        with tempfile.NamedTemporaryFile(mode='w') as pxml:
            pxml.write("""
                <library path="foobar">
                    <class type="pkg::FooBar" base_class_type="pkg::World">
                        <description>baz</description>
                    </class>
                </library>
                """)
            pxml.flush()
            result = parse_plugins_description_xml(pxml.name)
        self.assertIn('plugin_libraries', result)
        self.assertEqual(['foobar'], result['plugin_libraries'])

    def test_class_libraries_one_library(self):
        with tempfile.NamedTemporaryFile(mode='w') as pxml:
            pxml.write("""
                <class_libraries>
                    <library path="foobar">
                        <class type="pkg::FooBar" base_class_type="pkg::World">
                            <description>foobar?</description>
                        </class>
                    </library>
                </class_libraries>
                """)
            pxml.flush()
            result = parse_plugins_description_xml(pxml.name)
        self.assertIn('plugin_libraries', result)
        self.assertEqual(['foobar'], result['plugin_libraries'])

    def test_class_libraries_two_library(self):
        with tempfile.NamedTemporaryFile(mode='w') as pxml:
            pxml.write("""
                <class_libraries>
                    <library path="foobar">
                        <class type="pkg::FooBar" base_class_type="pkg::World">
                            <description>foobar?</description>
                        </class>
                    </library>
                    <library path="bazfoo">
                        <class type="pkg::BazFoo" base_class_type="pkg::World">
                            <description>bazfoo?</description>
                        </class>
                    </library>
                </class_libraries>
                """)
            pxml.flush()
            result = parse_plugins_description_xml(pxml.name)
        self.assertIn('plugin_libraries', result)
        self.assertEqual(['foobar', 'bazfoo'], result['plugin_libraries'])


if __name__ == '__main__':
    unittest.main()
