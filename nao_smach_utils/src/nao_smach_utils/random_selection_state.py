#!/usr/bin/env python
import rospy

from smach import State
from random import randrange


class RandomSelectionFromPoolState(State):
    ''' pool is a list with the objects to be selected. Max_tries the maximum number of random tries before giving up and repeating the last one
        output_key selected_item is the randomly selected item from the list.
    '''
    def __init__(self, pool=None, max_tries=10):
        input_keys = []
        if not pool:
            input_keys = ['pool']

        State.__init__(self, outcomes=['succeeded'], output_keys=['selected_item'], input_keys=input_keys)
        self._pool = pool
        self._lenpool = len(pool)
        self._lastSelection = None
        self._MAX_TRIES = max_tries

    def execute(self, userdata):
        lenpool = self._lenpool if self._pool else len(userdata.pool)
        selection = randrange(lenpool)
        if lenpool > 1:
            i = 0
            while (selection == self._lastSelection) and (i < self._MAX_TRIES):
                selection = randrange(lenpool)
                i += 1
            if i >= self._MAX_TRIES:
                rospy.logwarn('Maximum number of random tries (%d) reached! You may consider augmenting it...' % self._MAX_TRIES)

        self._lastSelection = selection
        userdata.selected_item = self._pool[selection] if self._pool else userdata.pool[selection]
        return 'succeeded'
