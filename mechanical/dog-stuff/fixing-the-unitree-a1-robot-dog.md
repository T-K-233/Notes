# Fixing the Unitree A1 Robot Dog Ethernet Port

So the two Ethernet ports on our A1 robot are broken when running some unstable RL algorithm experiments. And I was asked to fix them.

<figure><img src="../../.gitbook/assets/289cb20842797d1c96c2effc2c938bc.jpg" alt=""><figcaption><p>The dog. Note that the Ethernet ports are crippled</p></figcaption></figure>

## Disassembly

A1 is a bit more complicated compared to MIT Mini Cheetah. In order to take the top shell off, we need to remove:

* Screws on the top of the body
* Screws on the bottom side of the body
* Top half of the screws mounting the motor

I accidentally removed all of the screws mounting the rear motors, so the rear legs fall off.

<figure><img src="../../.gitbook/assets/48be338cfcca5ba88e222bec45c9c63.jpg" alt=""><figcaption></figcaption></figure>

The signal cables are not very long, and thus I ripped off the connector. Fortunately, the signal pads and connector itself are not damaged, so I was able to solder it back.

<figure><img src="../../.gitbook/assets/b2bd8b650df0b761f04fd0ad8d83d8a.jpg" alt=""><figcaption></figcaption></figure>

## Components

<figure><img src="../../.gitbook/assets/4d50fbe93955f14d75c52febe18c368.jpg" alt=""><figcaption><p>top shell taken off</p></figcaption></figure>

<figure><img src="../../.gitbook/assets/a8e2b8a597905e5a7dd7d2bbb484f77.jpg" alt=""><figcaption><p>High level controller board</p></figcaption></figure>

<figure><img src="../../.gitbook/assets/38917608aa243449d6554e677f909e3.jpg" alt=""><figcaption><p>Power distribution board</p></figcaption></figure>

<figure><img src="../../.gitbook/assets/aeecbd0dfec54bd89137f9449cceaf6.jpg" alt=""><figcaption><p>Not sure what this board is for</p></figcaption></figure>

<figure><img src="../../.gitbook/assets/0b3c5bf3ed19c957c60a9ccd139357b.jpg" alt=""><figcaption><p>Low level controller board</p></figcaption></figure>

To desolder and resolder the Ethernet port, we need to take the board out. In order to know which wire connects to where, I marked each connector with a unique marking with a Sharpie.

<figure><img src="../../.gitbook/assets/005aa53b609b94c7183d03fdb90cb8b.jpg" alt=""><figcaption></figcaption></figure>

It's interesting to see that they conformal coated the entire board to make it waterproof.

<figure><img src="../../.gitbook/assets/25819c5d6003275350c24d6a23f13db.jpg" alt=""><figcaption><p>High level controller board taken out</p></figcaption></figure>

<figure><img src="../../.gitbook/assets/5eea29a3e0c8442efb0413ec9ae5905.jpg" alt=""><figcaption><p>High level controller board, back side</p></figcaption></figure>

When taking out the low level controller board, pay special attention to the WiFi MHF4 connector. It is very fragile and easy to tear the pad off.

<figure><img src="../../.gitbook/assets/7c4dbb54043d1729914031683dde5a4.jpg" alt=""><figcaption><p>MHF4 connector</p></figcaption></figure>

<figure><img src="../../.gitbook/assets/b858dfc706245b1e1f7bb985ff41003.jpg" alt=""><figcaption><p>Similar marking on the low level controller board</p></figcaption></figure>

<figure><img src="../../.gitbook/assets/1d956998f3ce0ebb701e5660e33b940.jpg" alt=""><figcaption><p>Low level controller board taken out</p></figcaption></figure>

<figure><img src="../../.gitbook/assets/e7f58dc2428a7b5213f59dbdde6fa9e.jpg" alt=""><figcaption><p>Low level controller board, back side</p></figcaption></figure>

## The Ethernet Jack

Unfortunately, our lab bought the wrong replacement Ethernet jack.

<figure><img src="../../.gitbook/assets/2f3542e0b919805b07e97a389dff27a.jpg" alt=""><figcaption></figcaption></figure>

Found a similar enough Ethernet jack:

{% embed url="https://www.digikey.com/en/products/detail/te-connectivity-amp-connectors/1734795-4/5264486" %}

We still need to make a small modification to this part. Because the housing pin is wider and smaller on the A1 board footprint, we need to straighten the housing pin on the connector, and fold it.

<figure><img src="../../.gitbook/assets/cf61de32ace847bcf66459bc14e6773.jpg" alt=""><figcaption><p>left side: before modification; right side: after modification</p></figcaption></figure>

<figure><img src="../../.gitbook/assets/7e4aac21ad6c2d7089c2ee4fbff6899.jpg" alt=""><figcaption><p>After modification it fits the footprint</p></figcaption></figure>

<figure><img src="../../.gitbook/assets/6332e28234f4308f49e7d5865f15c71.jpg" alt=""><figcaption><p>Soldered and assebled</p></figcaption></figure>

## It worked

(at least I did not damage the rest of the components...)

<figure><img src="../../.gitbook/assets/983200db067d427b0fc2a6326d7a673.jpg" alt=""><figcaption></figcaption></figure>
