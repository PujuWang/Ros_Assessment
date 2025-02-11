from setuptools import find_packages, setup
import glob

package_name = 'bot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/robot_description',glob.glob('robot_description/*')),
        ('share/' + package_name + '/launch',glob.glob('launch/*')),
        ('share/' + package_name + '/config',glob.glob('config/*')),
        ('share/' + package_name + '/worlds',glob.glob('worlds/*')),
        ('share/' + package_name + '/floor_plan',glob.glob('floor_plan/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='edith',
    maintainer_email='edith@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "bot_navigator=bot_pkg.bot_navigator:main",
            "order_manager=bot_pkg.order_manager:main",
            "order_publisher=bot_pkg.order_publisher:main",
        ],
    },
)
