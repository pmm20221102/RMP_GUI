from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rmp_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # 1. 注册包名 (resource 文件夹现在有了，这行代码生效了)
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        # 2. 安装 package.xml
        ('share/' + package_name, ['package.xml']),
        
        # 3. 安装 launch 文件夹下的所有 .py 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # 4. 安装 config 文件夹
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nenlian',
    maintainer_email='nenlian@todo.todo',
    description='RMP Robot GUI',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'start_gui = rmp_gui.rmp_gui_main:main',
        ],
    },
)