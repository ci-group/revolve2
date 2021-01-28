import numpy
import struct
import binascii


# sensors for the controllers
class Sensors:

    MAX_SENSORS = 20

    def get_position(self, first, second):
        return first + ((second - first) / 2)

    def get(self, player, opponent, inputs_coded: bool = False):
        # calculates vertical and horizontal distances between sprites centers
        posx_p = self.get_position(player.rect.left, player.rect.right)
        posy_p = self.get_position(player.rect.bottom, player.rect.top)
        posx_e = self.get_position(opponent.rect.left, opponent.rect.right)
        posy_e = self.get_position(opponent.rect.bottom, opponent.rect.top)

        param_values = [posx_p-posx_e, posy_p-posy_e, player.direction, opponent.direction]

        # calculates vertical and horizontal distances between player and the center of enemy's bullets
        for projectile_key in sorted(list(opponent.projectiles.keys())):
            projectile = opponent.projectiles[projectile_key]
            posx_be = self.get_position(projectile.rect.left, projectile.rect.right)
            posy_be = self.get_position(projectile.rect.bottom, projectile.rect.top)
            param_values.append(posx_p - posx_be)
            param_values.append(posy_p - posy_be)

        # treats cases when not all bullets are used
        for i in range(len(param_values)-1, self.MAX_SENSORS-1):
            param_values.append(0)


        # applies several transformations to input variables (sensors)
        if inputs_coded:

            types = struct.Struct('q q q q q q q q q q q q q q q q q q q q') # defines the data types of each item of the array that will be packed. (q=int, f=flo)
            packed_data = types.pack(*param_values)  # packs data as struct
            coded_variables = binascii.hexlify(packed_data)  # converts packed data to an hexadecimal string
            coded_variables = [coded_variables[i:i+2] for i in range(0, len(coded_variables), 2)] # breaks hexadecimal string in bytes.
            coded_variables = numpy.array(map(lambda y: int(y, 16), coded_variables))  # converts bytes to integer

            param_values = coded_variables

        return numpy.array(param_values)
