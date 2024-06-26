from setuptools import find_packages, setup

package_name = 'ransac_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='osw',
    maintainer_email='oswystk15662@keio.jp',
    description='plz run sick scan xd before',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ransac_py_node = ransac_py.ransac_py_node:main',
        ],
    },
)
