"""python script for returning default logger to be used across modules"""

import logging

def return_logger(log_path, logger_write_mode="a"):
    """
    return standard logger to be used across modules
    
    write modes are:
        `'a'` = append to log file

        `'w'` = erase previous log and make new log file
    """
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.DEBUG)

    #%(asctime)s:
    formatter = logging.Formatter('%(module)s.%(funcName)s: %(message)s')
    file_handler = logging.FileHandler(log_path, mode=logger_write_mode)
    
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(formatter)

    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(formatter)

    logger.addHandler(file_handler)
    logger.addHandler(stream_handler)

    return logger