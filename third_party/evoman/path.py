import os
import rootpath

evoman_path = os.path.join(rootpath.detect(), 'third_party', 'evoman')
map_path = os.path.join(evoman_path, 'resources', 'map')
image_path = os.path.join(evoman_path, 'resources', 'images')
projectile_image_path = os.path.join(image_path, 'projectile')
sound_path = os.path.join(evoman_path, 'resources', 'sounds')
experiment_path = os.path.join(evoman_path, "test")