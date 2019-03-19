# SigFox mailbox notifier

This is a basic Arduino sketch for the MkrFox1200 that sends a message over the SigFox network whenever the mailbox is filled (in other words, "you've got mail") or when it has been emptied. Everything is triggered using two reed switches.

Includes a "hacky debounce" to minimise the risk of false positives.

Lastly, the module also transmits the current battery voltage and the on-board temperature of the SigFox module (which can be used to approximate the outdoor temperature depending of the mailbox location and exposure to the sun).

