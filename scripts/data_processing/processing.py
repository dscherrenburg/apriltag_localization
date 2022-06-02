def calculate_mean_std(data):
    """
    Calculate mean and standard deviation of the data.
    :param data: data to calculate mean and standard deviation
    :return: mean and standard deviation of the data
    """
    mean = np.mean(data)
    std = np.std(data)
    return mean, std