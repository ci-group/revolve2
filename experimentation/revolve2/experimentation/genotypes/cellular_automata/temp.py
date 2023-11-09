import random

def mutate():
    # Define the possible values
    possible_values = [0.0, 1.0, 2.0]

    # Create an example dictionary
    gene = {
        (1.0, 0.0, 0.0, 0.0): 1.0,
        (0.0, 1.0, 0.0, 0.0): 1.0,
        (0.0, 0.0, 1.0, 0.0): 1.0,
        (0.0, 0.0, 0.0, 1.0): 1.0,
    }

    print(gene)

    # Generate a random key-value pair
    new_key = tuple(random.choice(possible_values) for _ in range(4))
    new_value = random.choice(possible_values)

    # Choose an existing key to replace
    existing_key_index = random.randint(0, len(gene) - 1)
    existing_key = list(gene.keys())[existing_key_index]

    # Update the dictionary with the new key-value pair while replacing the existing one
    gene[new_key] = new_value
    del gene[existing_key]

    return gene



def random_genotype(n):
    possible_values = [0.0, 1.0, 2.0]
    gene = {}

    for i in range(n):
        # Generate n random key value pairs
        new_key = tuple(random.choice(possible_values) for _ in range(4))
        new_value = random.choice(possible_values)
        gene[new_key] = new_value

    return gene


def robot_from_ca_test():

    print(random_genotype(4))


if __name__ == '__main__':
    robot_from_ca_test()