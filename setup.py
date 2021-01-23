from setuptools import setup

setup(name='panda_gym_env',
      version='0.0.1',
      description='OpenAI Gym environment for panda',
      author='Jonas Springer',
      url="",
      packages=setuptools.find_packages(include="gym_panda*"),
      install_requires=['gym', 'numpy'],
      python_requires='>=3.6'
)
