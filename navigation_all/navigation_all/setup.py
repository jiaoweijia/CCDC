from setuptools import setup, find_packages,find_namespace_packages

setup(
    name="navigation_global",
    version="0.1",
    #packages=find_packages(),
    packages=find_namespace_packages(include=[
        'navigation_global.*',
        'nav_utils.*', 
        'geo_tools.*',
        'common.*'
    ]),
)