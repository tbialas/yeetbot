# A response should only be sent after the user has chosen an option from
# a prior list of YEETBotUserChoices.

# Which number the choice is, corresponsing to the prior list of choices
int32 choice

# Set this to true if the user somehow selected a choice which is different
# to one of the acceptable answers. This might be useful for NLP?
# Set it to false otherwise.
bool invalid_choice
