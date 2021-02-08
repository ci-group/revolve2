import imageio


class ImageAnimator:

    @staticmethod
    def from_files(filenames, animation_filename):
        images = []
        for filename in filenames:
            images.append(imageio.imread(filename))

        imageio.mimsave(animation_filename, images)
