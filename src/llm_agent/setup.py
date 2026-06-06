from setuptools import find_packages, setup
from glob import glob

package_name = 'llm_agent'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'pyyaml', 'google-genai', 'python-dotenv'],
    zip_safe=True,
    maintainer='psj',
    maintainer_email='psj15641@gmail.com',
    description='LLM-based agent for natural language robot Pick and Place in ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agent_node = llm_agent.agent_node:main',
        ],
    },
)
