from setuptools import find_packages, setup

package_name = 'humanoid_agent'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rabikes',
    maintainer_email='you@example.com',
    description='AI agent pre robota Chappie (kamera + LLM -> skills)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'agent_node = humanoid_agent.agent_node:main',
            'task_client = humanoid_agent.task_client:main',
        ],
    },
)
