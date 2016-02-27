#!/usr/bin/env python
"""
Scraper for Doug McKenzie Jazz Piano MIDI recordings
"""
import urllib2
import urllib
from BeautifulSoup import BeautifulSoup

req = urllib2.Request("http://www.bushgrafts.com/jazz/midi.htm")
response = urllib2.urlopen(req)
page_contents = response.read()

base_curl_url = "http://www.bushgrafts.com/jazz/"

soup = BeautifulSoup(page_contents)
for link in soup.findAll('a', href=True):
	if link['href'].endswith('.mid'):
		print "Scraping " + link['href'].split('/')[1]
		full_url = base_curl_url + urllib.quote(link['href'])
		read_file = urllib.URLopener()
		read_file.retrieve(full_url, "mckenzie/" + link['href'].split('/')[1])


