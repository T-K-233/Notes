# Gitbook Errata



## Images

When uploading image, Gitbook will insert the image at the cursor **at the time when the image finishes uploading**.

This might cause some issue, especially when internet connection is slow, if you upload an image and then immediately proceed to edit something else.

Solution: keep the cursor at the location until upload finishes and the image is shown.



## Code block&#x20;

If the code block is inserted as the last block in the page, there is no way to exit the code block and create other blocks below that.

Solution:&#x20;

When editing a page, keep several blank lines below just for reserve. I normally do 5-8 empty lines.

If you are indeed stuck at the code block, create a new text block above the code block with some random text, then drag the text block, trigger the reorder functionality, and then put the text block below the code block.&#x20;

Same stuff applies to inline code formatting and tables.



## Tabs

In tabs, when you try to create more than 1 empty lines by pressing Enter, it will register the action as exit out of the block, and move the cursor outside of the tab.

To create empty lines, type some random characters in each line to prevent this behavior.

Normally I would do something like this:

{% tabs %}
{% tab title="First Tab" %}
1

2

3

4

5
{% endtab %}

{% tab title="Second Tab" %}
a

a

a

a
{% endtab %}
{% endtabs %}









