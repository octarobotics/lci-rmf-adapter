from setuptools import find_packages, setup

package_name = 'lci_rmf_adapter'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cota Nabeshima',
    maintainer_email='cota@octa8.jp',
    description='LCI client as Open-RMF Lift and Door Adapter',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lci_rmf_adapter = lci_rmf_adapter.lci_rmf_adapter:main',
            'lci_rmf_adapter_lift_test = lci_rmf_adapter.lci_rmf_adapter_lift_test:main',
            'lci_rmf_adapter_door_test = lci_rmf_adapter.lci_rmf_adapter_door_test:main',            
        ],
    },
)
