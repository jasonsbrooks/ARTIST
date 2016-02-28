#!/usr/bin/env python
"""
Scraper for Doug McKenzie Jazz Piano MIDI recordings
"""
import urllib2
import urllib
from BeautifulSoup import BeautifulSoup


for i in xrange(1,7):
	url = "http://www.ajsmidi.com/jazz/jazz%01d.html" % i
	req = urllib2.Request(url)
	response = urllib2.urlopen(req)
	page_contents = response.read()

	base_curl_url = "http://www.ajsmidi.com/jazz/midi/"

	soup = BeautifulSoup(page_contents)
	for link in soup.findAll('a', href=True):
		if link['href'].endswith('.mid'):
			print "Scraping " + link['href'].split('/')[-1]
			full_url = base_curl_url + urllib.quote(link['href'].split('/')[-1])
			read_file = urllib.URLopener()
			try:
				read_file.retrieve(full_url, "ajsmidi/" + link['href'].split('/')[-1])
			except IOError, e:
				print e
