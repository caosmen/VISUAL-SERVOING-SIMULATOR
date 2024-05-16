from setuptools import find_packages, setup

package_name = 'visual_servoing_calibrated'

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
    maintainer='caosmen',
    maintainer_email='bll@ic.ufal.br',
    description='Visual servoing with a calibrated camera',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visual_servoing_calibrated = visual_servoing_calibrated.visual_servoing_calibrated:main',
        ],
    },
)
