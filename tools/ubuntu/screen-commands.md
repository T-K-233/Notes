# Screen Commands



### In normal terminal

<table><thead><tr><th width="361">Command</th><th>Description</th></tr></thead><tbody><tr><td><code>screen -S &#x3C;sessionname></code></td><td>Start a new screen session with a given name and attach to it.</td></tr><tr><td><code>screen -ls</code></td><td>Lists all existing screen sessions.</td></tr><tr><td><code>screen -r</code></td><td>Reattach this terminal to the only existing screen session.</td></tr><tr><td><code>screen -r -S &#x3C;sessionname></code></td><td>Reattach this terminal to an existing screen session by name.</td></tr><tr><td><code>screen -S &#x3C;oldname> -X &#x3C;newname></code></td><td>Rename a screen session.</td></tr><tr><td><code>screen -S &#x3C;sessionname> -X quit</code></td><td>Kill the specified screen session by executing quit.</td></tr></tbody></table>



### Within a screen session

Prepend every command with `ctrl` + `A`

<table><thead><tr><th width="243">Command</th><th>Description</th></tr></thead><tbody><tr><td><code>Ctrl</code>+<code>A</code> - <code>?</code></td><td>Show help</td></tr><tr><td><code>Ctrl</code>+<code>A</code> - <code>d</code></td><td><strong>Detach</strong> the current screen session from this terminal. The screen session and its processes remain in existence.</td></tr><tr><td><code>Ctrl</code>+<code>A</code> - <code>c</code></td><td><strong>Create</strong> a new window and switch to it.</td></tr><tr><td><code>Ctrl</code>+<code>A</code> - <code>k</code> (y)</td><td><strong>Kill</strong> the current window.</td></tr><tr><td></td><td></td></tr><tr><td><code>Ctrl</code>+<code>A</code> - <code>p</code></td><td>Go to the <strong>previous</strong> screen.</td></tr><tr><td><code>Ctrl</code>+<code>A</code> - <code>n</code></td><td>Go to the <strong>next</strong> screen.</td></tr><tr><td><code>Ctrl</code>+<code>A</code> - <code>&#x3C;number></code></td><td>Go to a specific screen <strong>number.</strong></td></tr><tr><td><code>Ctrl</code>+<code>A</code> - <code>"</code></td><td>Go to the screen selection page. Select a screen from the list using the arrow keys.</td></tr><tr><td></td><td></td></tr><tr><td><code>Ctrl</code>+<code>A</code> - <code>s</code></td><td><strong>Split</strong> the current window in half horizontally.</td></tr><tr><td><code>Ctrl</code>+<code>A</code> - <code>|</code></td><td><strong>Split</strong> the current window in half vertically.</td></tr><tr><td><code>Ctrl</code>+<code>A</code> - <code>TAB</code></td><td><strong>Cycle</strong> through window regions.</td></tr><tr><td><code>Ctrl</code>+<code>A</code> - <code>X</code></td><td><strong>Eliminate</strong> a window split.</td></tr></tbody></table>



See if you are inside a screen

```bash
echo $TERM
```



normal ssh session:

```bash
$ echo $TERM
xterm-256color
```

inside screen:

```bash
$ echo $TERM
screen.xterm-256color
```
